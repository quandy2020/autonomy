/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/follow_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

namespace {

namespace task_proto = ::autonomy::task::proto;
namespace follow_rpc = ::automsgs::rpcs::follow;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

bool CheckTrackerTerminalStatus(task_proto::TrackerStatus status) {
    return status == task_proto::TRACKER_STATUS_SUCCEEDED ||
           status == task_proto::TRACKER_STATUS_FAILED ||
           status == task_proto::TRACKER_STATUS_CANCELED;
}

follow_rpc::FollowState ToFollowState(task_proto::TrackerStatus status) {
    switch (status) {
        case task_proto::TRACKER_STATUS_IDLE:
            return follow_rpc::FOLLOW_STATE_IDLE;
        case task_proto::TRACKER_STATUS_TRACKING:
            return follow_rpc::FOLLOW_STATE_FOLLOWING;
        case task_proto::TRACKER_STATUS_PAUSED:
            return follow_rpc::FOLLOW_STATE_PAUSED;
        case task_proto::TRACKER_STATUS_TARGET_LOST:
            return follow_rpc::FOLLOW_STATE_LOST_TARGET;
        case task_proto::TRACKER_STATUS_FAILED:
            return follow_rpc::FOLLOW_STATE_FAILED;
        case task_proto::TRACKER_STATUS_CANCELED:
            return follow_rpc::FOLLOW_STATE_CANCELLED;
        case task_proto::TRACKER_STATUS_SUCCEEDED:
            return follow_rpc::FOLLOW_STATE_IDLE;
        default:
            return follow_rpc::FOLLOW_STATE_UNKNOWN;
    }
}

}  // namespace

FollowTraits::Goal FollowTraits::ConvertToGoal(const Request& request) {
    Goal goal;
    goal.set_command(task_proto::TRACKER_CMD_START);
    switch (request.target_type()) {
        case follow_rpc::FOLLOW_TARGET_TYPE_PERSON:
            goal.set_mode(task_proto::TRACKER_MODE_PERSON);
            break;
        case follow_rpc::FOLLOW_TARGET_TYPE_CUSTOM:
            goal.set_mode(task_proto::TRACKER_MODE_TARGET_ID);
            break;
        default:
            goal.set_mode(request.target_id().empty()
                              ? task_proto::TRACKER_MODE_PERSON
                              : task_proto::TRACKER_MODE_TARGET_ID);
            break;
    }
    if (!request.target_id().empty()) {
        goal.set_target_id(request.target_id());
    }
    if (request.has_hint_pose()) {
        auto* pose = goal.mutable_target_pose();
        *pose->mutable_pose() = request.hint_pose();
        if (request.has_header()) {
            *pose->mutable_header() = request.header();
        }
    }
    if (request.has_options() && request.options().has_desired_distance_m()) {
        goal.set_follow_distance(request.options().desired_distance_m());
    }
    goal.set_reacquire_on_lost(true);
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_FOLLOW);
    return goal;
}

FollowTraits::Response FollowTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    Response response;
    response.set_goal_id(last.goal_id());
    response.set_target_type(last.target_type());
    response.set_target_id(last.target_id());
    response.set_distance_m(feedback.distance_to_target());
    response.set_state(ToFollowState(feedback.status()));
    const bool terminal = CheckTrackerTerminalStatus(feedback.status());
    response.set_active(!terminal);
    if (feedback.status() == task_proto::TRACKER_STATUS_SUCCEEDED) {
        response.set_state(follow_rpc::FOLLOW_STATE_IDLE);
    }
    const bool ok = !terminal ||
                    feedback.status() == task_proto::TRACKER_STATUS_SUCCEEDED;
    *response.mutable_status() =
        ok ? OkStatus()
           : ErrorStatus(StatusCode::FOLLOW_BUSY, response.message());
    return response;
}

FollowTraits::Response FollowTraits::MakeResponse(const Request& request,
                                                  bool success, bool final,
                                                  const std::string& message) {
    Response response;
    response.set_goal_id(request.goal_id());
    response.set_target_type(request.target_type());
    response.set_target_id(request.target_id());
    response.set_message(message);
    if (success) {
        response.set_state(final ? follow_rpc::FOLLOW_STATE_CANCELLED
                                 : follow_rpc::FOLLOW_STATE_FOLLOWING);
        response.set_active(!final);
        *response.mutable_status() = OkStatus(message);
    } else {
        response.set_state(follow_rpc::FOLLOW_STATE_FAILED);
        response.set_active(false);
        *response.mutable_status() =
            ErrorStatus(StatusCode::FOLLOW_BUSY, message);
    }
    return response;
}

bool FollowTraits::IsTerminal(const Feedback& feedback) {
    return CheckTrackerTerminalStatus(feedback.status());
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
