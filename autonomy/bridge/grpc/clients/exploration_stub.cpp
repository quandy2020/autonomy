/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/exploration_stub.hpp"

#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/common/names.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;

proto::TaskStatus ResolveExplorationTaskStatus(
    proto::ExplorationStatus status) {
    switch (status) {
        case proto::EXPLORATION_STATUS_PLANNING:
        case proto::EXPLORATION_STATUS_EXPLORING:
            return proto::TASK_STATUS_RUNNING;
        case proto::EXPLORATION_STATUS_PAUSED:
            return proto::TASK_STATUS_PAUSED;
        case proto::EXPLORATION_STATUS_SUCCEEDED:
            return proto::TASK_STATUS_SUCCEEDED;
        case proto::EXPLORATION_STATUS_FAILED:
            return proto::TASK_STATUS_FAILED;
        case proto::EXPLORATION_STATUS_CANCELED:
            return proto::TASK_STATUS_CANCELED;
        default:
            return proto::TASK_STATUS_IDLE;
    }
}

}  // namespace

ExplorationStub::ExplorationStub(std::shared_ptr<autolink::Node> node,
                                 std::shared_ptr<TaskMuxer> muxer)
    : node_(std::move(node)), muxer_(std::move(muxer)) {
    if (!node_) {
        return;
    }
    mapping_writer_ =
        node_->CreateWriter<task_proto::MappingGoal>(::autonomy::task::kMappingGoal);
    waypoint_writer_ =
        node_->CreateWriter<::automsgs::msgs::geometry_msgs::PoseStamped>(
            ::autonomy::task::kExplorationWaypoint);
    finished_reader_ = node_->CreateReader<::automsgs::msgs::std_msgs::Bool>(
        ::autonomy::task::kExplorationFinished,
        [this](const std::shared_ptr<::automsgs::msgs::std_msgs::Bool>& message) {
            HandleFinished(message);
        });
    mapping_feedback_reader_ = node_->CreateReader<task_proto::MappingFeedback>(
        ::autonomy::task::kMappingFeedback,
        [this](const std::shared_ptr<task_proto::MappingFeedback>& feedback) {
            HandleMappingFeedback(feedback);
        });
}

void ExplorationStub::CancelActiveSession() {
    proto::ExplorationCommandRequest last;
    auto callback = session_.TakeCallback(true, &last);
    if (callback) {
        status_ = proto::EXPLORATION_STATUS_CANCELED;
        callback(MakeResponse(last, status_, true, true, "exploration cancelled"));
    }
    if (muxer_) {
        muxer_->Release(proto::TASK_TYPE_EXPLORATION);
    }
}

bool ExplorationStub::PublishMappingGoal(const task_proto::MappingGoal& goal) {
    return mapping_writer_ && mapping_writer_->Write(goal);
}

proto::ExplorationCommandResponse ExplorationStub::MakeResponse(
    const proto::ExplorationCommandRequest& request,
    const proto::ExplorationStatus status, const bool success, const bool final,
    const std::string& message) const {
    proto::ExplorationCommandResponse response;
    response.set_status(status);
    response.set_progress(progress_);
    response.set_map_name(map_name_.empty() ? request.map_name() : map_name_);
    FillCommandAck(response, proto::TASK_TYPE_EXPLORATION, request, success,
                   final, ResolveExplorationTaskStatus(status), message);
    return response;
}

proto::ExplorationCommandResponse ExplorationStub::GetSnapshot() const {
    return MakeResponse(session_.GetLastRequest(), status_, true, false);
}

void ExplorationStub::HandleFinished(
    const std::shared_ptr<::automsgs::msgs::std_msgs::Bool>& message) {
    if (!message || !message->data()) {
        return;
    }
    proto::ExplorationCommandRequest last;
    auto callback = session_.TakeCallback(true, &last);
    if (!callback) {
        return;
    }
    status_ = proto::EXPLORATION_STATUS_SUCCEEDED;
    progress_ = 1.f;
    callback(MakeResponse(last, status_, true, true, "exploration finished"));
    if (muxer_) {
        muxer_->Release(proto::TASK_TYPE_EXPLORATION);
    }
}

void ExplorationStub::HandleMappingFeedback(
    const std::shared_ptr<task_proto::MappingFeedback>& feedback) {
    if (!feedback || !session_.CheckSessionActive()) {
        return;
    }
    if (!feedback->current_map_name().empty()) {
        map_name_ = feedback->current_map_name();
    }
    if (feedback->progress().progress() > 0.f) {
        progress_ = feedback->progress().progress();
    }
    status_ = proto::EXPLORATION_STATUS_EXPLORING;
    session_.EmitResponse(
        MakeResponse(session_.GetLastRequest(), status_, true, false));
}

bool ExplorationStub::HandleCommand(
    const proto::ExplorationCommandRequest& request,
    StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    auto fail = [&](const std::string& message) {
        stream_callback(MakeResponse(request, proto::EXPLORATION_STATUS_FAILED,
                                     false, true, message));
    };
    if (RejectIfEstopActive(muxer_, fail)) {
        return false;
    }

    const auto command = request.command();
    if (command == proto::EXPLORATION_CMD_START) {
        if (RejectIfAcquireFailed(muxer_, proto::TASK_TYPE_EXPLORATION, request,
                                  fail)) {
            return false;
        }
    }

    session_.BindStream(request, stream_callback);
    if (!request.map_name().empty()) {
        map_name_ = request.map_name();
    }

    const bool handled = DispatchCommands(
        command,
        MakeCommandRule(proto::EXPLORATION_CMD_START, [&] {
            if (request.enable_mapping()) {
                task_proto::MappingGoal goal;
                goal.set_command(task_proto::MAP_CMD_LOAD);
                if (!request.map_name().empty()) {
                    goal.set_map_name(request.map_name());
                }
                if (request.has_header()) {
                    auto* header = goal.mutable_header();
                    header->set_task_id(request.header().cmd_id());
                    header->set_client_id(request.header().client_id());
                    header->set_task_type(
                        ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_EXPLORATION);
                }
                PublishMappingGoal(goal);
            }
            if (request.has_area() && request.area().points_size() > 0 &&
                waypoint_writer_) {
                ::automsgs::msgs::geometry_msgs::PoseStamped waypoint;
                double x = 0., y = 0.;
                for (const auto& p : request.area().points()) {
                    x += p.x();
                    y += p.y();
                }
                const auto n = static_cast<double>(request.area().points_size());
                waypoint.mutable_pose()->mutable_position()->set_x(x / n);
                waypoint.mutable_pose()->mutable_position()->set_y(y / n);
                waypoint.mutable_pose()->mutable_orientation()->set_w(1.0);
                waypoint_writer_->Write(waypoint);
            }
            session_.SetSessionActive(true);
            status_ = proto::EXPLORATION_STATUS_EXPLORING;
            progress_ = 0.f;
            stream_callback(MakeResponse(
                request, proto::EXPLORATION_STATUS_EXPLORING, true, false));
            return true;
        }),
        MakeCommandRule(proto::EXPLORATION_CMD_PAUSE, [&] {
            status_ = proto::EXPLORATION_STATUS_PAUSED;
            stream_callback(MakeResponse(
                request, proto::EXPLORATION_STATUS_PAUSED, true, false));
            return true;
        }),
        MakeCommandRule(proto::EXPLORATION_CMD_RESUME, [&] {
            session_.SetSessionActive(true);
            status_ = proto::EXPLORATION_STATUS_EXPLORING;
            stream_callback(MakeResponse(
                request, proto::EXPLORATION_STATUS_EXPLORING, true, false));
            return true;
        }),
        MakeCommandRule(proto::EXPLORATION_CMD_SAVE_MAP, [&] {
            task_proto::MappingGoal goal;
            goal.set_command(task_proto::MAP_CMD_SWITCH);
            goal.set_map_name(request.map_name().empty() ? map_name_
                                                         : request.map_name());
            PublishMappingGoal(goal);
            stream_callback(MakeResponse(request, status_, true, false,
                                         "save map requested"));
            return true;
        }),
        MakeCommandRule(proto::EXPLORATION_CMD_SET_AREA, [&] {
            stream_callback(MakeResponse(request, status_, true, false,
                                         "exploration area updated"));
            return true;
        }),
        MakeCommandRules(
            [&] {
                CancelActiveSession();
                stream_callback(MakeResponse(
                    request, proto::EXPLORATION_STATUS_CANCELED, true, true));
                return true;
            },
            proto::EXPLORATION_CMD_STOP, proto::EXPLORATION_CMD_CANCEL));

    if (handled) {
        return true;
    }
    fail("unsupported exploration command");
    return false;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
