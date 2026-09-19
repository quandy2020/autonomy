/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/exploration_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;
namespace explore_rpc = ::automsgs::rpcs::exploration;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

bool IsExplorationTerminalStatus(task_proto::ExplorationStatus status) {
    return status == task_proto::EXPLORATION_STATUS_SUCCEEDED ||
           status == task_proto::EXPLORATION_STATUS_FAILED ||
           status == task_proto::EXPLORATION_STATUS_CANCELED;
}

explore_rpc::ExplorationState ToRpcState(
    task_proto::ExplorationStatus status) {
    switch (status) {
        case task_proto::EXPLORATION_STATUS_IDLE:
            return explore_rpc::EXPLORATION_STATE_IDLE;
        case task_proto::EXPLORATION_STATUS_PLANNING:
            return explore_rpc::EXPLORATION_STATE_PLANNING;
        case task_proto::EXPLORATION_STATUS_EXPLORING:
            return explore_rpc::EXPLORATION_STATE_EXPLORING;
        case task_proto::EXPLORATION_STATUS_PAUSED:
            return explore_rpc::EXPLORATION_STATE_PAUSED;
        case task_proto::EXPLORATION_STATUS_SUCCEEDED:
            return explore_rpc::EXPLORATION_STATE_COMPLETED;
        case task_proto::EXPLORATION_STATUS_FAILED:
            return explore_rpc::EXPLORATION_STATE_FAILED;
        case task_proto::EXPLORATION_STATUS_CANCELED:
            return explore_rpc::EXPLORATION_STATE_CANCELLED;
        default:
            return explore_rpc::EXPLORATION_STATE_UNKNOWN;
    }
}

}  // namespace

bool ExplorationStub::HandleExplore(
    const explore_rpc::ExploreRequest& request,
    StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    return HandleRequest(
        request, [this, cb = std::move(stream_callback)](const Response& r) {
            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                last_response_ = r;
            }
            cb(r);
        });
}

ExplorationTraits::Goal ExplorationTraits::ConvertToGoal(
    const Request& request) {
    Goal goal;
    goal.set_command(task_proto::EXPLORATION_CMD_START);
    if (request.has_area()) {
        *goal.mutable_area() = request.area();
    }
    if (!request.map_name().empty()) {
        goal.set_map_name(request.map_name());
    }
    if (request.has_options()) {
        goal.set_enable_mapping(request.options().enable_mapping());
        if (request.options().has_coverage_target()) {
            goal.set_coverage_target(request.options().coverage_target());
        }
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_EXPLORATION);
    return goal;
}

ExplorationTraits::Response ExplorationTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    Response response;
    response.set_goal_id(last.goal_id());
    response.set_state(ToRpcState(feedback.status()));
    response.set_map_name(feedback.map_name().empty() ? last.map_name()
                                                      : feedback.map_name());
    if (feedback.has_progress() && feedback.progress().progress() > 0.f) {
        response.set_progress(feedback.progress().progress());
    }
    if (feedback.explored_area() > 0.f) {
        response.set_explored_area_m2(feedback.explored_area());
    }
    const bool terminal = IsExplorationTerminalStatus(feedback.status());
    response.set_active(!terminal &&
                        feedback.status() != task_proto::EXPLORATION_STATUS_IDLE);
    const bool ok =
        !terminal ||
        feedback.status() == task_proto::EXPLORATION_STATUS_SUCCEEDED;
    *response.mutable_status() =
        ok ? OkStatus()
           : ErrorStatus(StatusCode::EXPLORATION_BUSY, response.detail());
    return response;
}

ExplorationTraits::Response ExplorationTraits::MakeResponse(
    const Request& request, bool success, bool final,
    const std::string& message) {
    Response response;
    response.set_goal_id(request.goal_id());
    response.set_map_name(request.map_name());
    response.set_detail(message);
    if (success) {
        response.set_state(final ? explore_rpc::EXPLORATION_STATE_CANCELLED
                                 : explore_rpc::EXPLORATION_STATE_EXPLORING);
        response.set_active(!final);
        *response.mutable_status() = OkStatus(message);
    } else {
        response.set_state(explore_rpc::EXPLORATION_STATE_FAILED);
        response.set_active(false);
        *response.mutable_status() =
            ErrorStatus(StatusCode::EXPLORATION_BUSY, message);
    }
    return response;
}

bool ExplorationTraits::IsTerminal(const Feedback& feedback) {
    return IsExplorationTerminalStatus(feedback.status());
}

void ExplorationStub::SetArea(
    const explore_rpc::SetAreaRequest& request) {
    task_proto::ExplorationGoal goal;
    goal.set_command(task_proto::EXPLORATION_CMD_SET_AREA);
    if (request.has_area()) {
        *goal.mutable_area() = request.area();
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_EXPLORATION);
    channel_.WriteGoal(goal);
}

void ExplorationStub::SaveMap(
    const explore_rpc::SaveMapRequest& request) {
    task_proto::ExplorationGoal goal;
    goal.set_command(task_proto::EXPLORATION_CMD_SAVE_MAP);
    if (!request.map_name().empty()) {
        goal.set_map_name(request.map_name());
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_EXPLORATION);
    channel_.WriteGoal(goal);
}

explore_rpc::ExploreResponse ExplorationStub::GetSnapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!last_response_.goal_id().empty() || last_response_.has_status()) {
        return last_response_;
    }
    return ExplorationTraits::MakeResponse(
        ::automsgs::rpcs::exploration::ExploreRequest{}, true, true, "");
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
