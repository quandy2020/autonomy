/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/mapping_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;
namespace mapping_rpc = ::automsgs::rpcs::mapping;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

bool CheckMapTerminalStatus(task_proto::MapStatus status) {
    return status == task_proto::MAP_STATUS_SUCCEEDED ||
           status == task_proto::MAP_STATUS_FAILED;
}

mapping_rpc::MappingState ToMappingState(task_proto::MapStatus status) {
    switch (status) {
        case task_proto::MAP_STATUS_SUCCEEDED:
            return mapping_rpc::MAPPING_STATE_IDLE;
        case task_proto::MAP_STATUS_FAILED:
            return mapping_rpc::MAPPING_STATE_FAILED;
        default:
            return mapping_rpc::MAPPING_STATE_MAPPING;
    }
}

}  // namespace

MappingStub::MappingStub(std::shared_ptr<autolink::Node> node,
                         TaskMuxer::SharedPtr muxer)
    : GoalChannelCommandStub(std::move(node), std::move(muxer)) {
    channel_.SetFeedbackHook(
        [this](const task_proto::MappingFeedback& feedback) {
            std::lock_guard<std::mutex> lock(map_name_mutex_);
            current_map_name_ = feedback.current_map_name();
        });
}

MappingTraits::Goal MappingTraits::ConvertToGoal(const Request& request) {
    Goal goal;
    goal.set_command(task_proto::MAP_CMD_LOAD);
    if (!request.map_name().empty()) {
        goal.set_map_name(request.map_name());
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_MAP);
    return goal;
}

MappingTraits::Response MappingTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    Response response;
    response.set_goal_id(last.goal_id());
    response.set_map_name(feedback.current_map_name().empty()
                              ? last.map_name()
                              : feedback.current_map_name());
    response.set_map_identifier(response.map_name());
    response.set_state(ToMappingState(feedback.status()));
    *response.mutable_status() =
        feedback.status() == task_proto::MAP_STATUS_FAILED
            ? ErrorStatus(StatusCode::MAPPING_BUSY, "mapping failed")
            : OkStatus();
    return response;
}

MappingTraits::Response MappingTraits::MakeResponse(const Request& request,
                                                    bool success, bool final,
                                                    const std::string& message) {
    Response response;
    response.set_goal_id(request.goal_id());
    response.set_map_name(request.map_name());
    response.set_map_identifier(request.map_name());
    response.set_state(success ? (final ? mapping_rpc::MAPPING_STATE_IDLE
                                        : mapping_rpc::MAPPING_STATE_MAPPING)
                               : mapping_rpc::MAPPING_STATE_FAILED);
    response.set_detail(message);
    *response.mutable_status() =
        success ? OkStatus(message)
                : ErrorStatus(StatusCode::MAPPING_BUSY, message);
    return response;
}

bool MappingTraits::IsTerminal(const Feedback& feedback) {
    return CheckMapTerminalStatus(feedback.status());
}

std::string MappingStub::GetCurrentMapName() const {
    std::lock_guard<std::mutex> lock(map_name_mutex_);
    return current_map_name_;
}

bool MappingStub::HandleStart(const mapping_rpc::StartMappingRequest& request,
                              StreamCallback stream_callback) {
    if (!request.map_name().empty()) {
        std::lock_guard<std::mutex> lock(map_name_mutex_);
        current_map_name_ = request.map_name();
    }
    if (!stream_callback) {
        stream_callback = [](const auto&) {};
    }
    const std::string map_name = GetCurrentMapName();
    return HandleRequest(
        request, [stream_callback = std::move(stream_callback),
                  map_name](const mapping_rpc::MappingStatus& response) {
            auto copy = response;
            if (copy.map_name().empty()) {
                copy.set_map_name(map_name);
            }
            stream_callback(copy);
        });
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
