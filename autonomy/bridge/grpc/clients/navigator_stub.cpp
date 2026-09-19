/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/navigator_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;
namespace nav_rpc = ::automsgs::rpcs::navigation;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

bool IsNavTerminalStatus(task_proto::NavigationStatus status) {
    // Match RPC IsNavigateTerminal: ARRIVED/SUCCEEDED keep the stream open.
    return status == task_proto::NAV_STATUS_FAILED ||
           status == task_proto::NAV_STATUS_CANCELED;
}

nav_rpc::NavigationState ToRpcState(task_proto::NavigationStatus status) {
    switch (status) {
        case task_proto::NAV_STATUS_IDLE:
            return nav_rpc::NAVIGATION_STATE_IDLE;
        case task_proto::NAV_STATUS_PLANNING:
            return nav_rpc::NAVIGATION_STATE_PLANNING;
        case task_proto::NAV_STATUS_NAVIGATING:
            return nav_rpc::NAVIGATION_STATE_RUNNING;
        case task_proto::NAV_STATUS_PAUSED:
            return nav_rpc::NAVIGATION_STATE_PAUSED;
        case task_proto::NAV_STATUS_SUCCEEDED:
            return nav_rpc::NAVIGATION_STATE_ARRIVED;
        case task_proto::NAV_STATUS_FAILED:
            return nav_rpc::NAVIGATION_STATE_FAILED;
        case task_proto::NAV_STATUS_CANCELED:
            return nav_rpc::NAVIGATION_STATE_CANCELLED;
        default:
            return nav_rpc::NAVIGATION_STATE_UNKNOWN;
    }
}

}  // namespace

NavigatorTraits::Goal NavigatorTraits::ConvertToGoal(const Request& request) {
    Goal goal;
    goal.set_command(task_proto::NAV_CMD_START);
    goal.set_mode(request.waypoints_size() > 1
                      ? task_proto::NAV_MODE_THROUGH_POSES
                      : task_proto::NAV_MODE_SINGLE_POSE);
    for (const auto& waypoint : request.waypoints()) {
        *goal.add_goals() = waypoint;
    }
    if (request.has_options() && request.options().has_timeout_seconds()) {
        goal.set_timeout_sec(request.options().timeout_seconds());
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NAVIGATION);
    return goal;
}

NavigatorTraits::Response NavigatorTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    Response response;
    response.set_goal_id(last.goal_id());
    response.set_state(ToRpcState(feedback.status()));
    response.set_waypoint_index(feedback.current_waypoint_index());
    response.set_number_of_waypoints(
        feedback.total_waypoints() > 0 ? feedback.total_waypoints()
                                       : last.waypoints_size());
    if (feedback.has_current_pose()) {
        *response.mutable_current_pose() = feedback.current_pose();
    }
    if (feedback.distance_remaining() > 0.f) {
        response.set_remaining_distance_meters(feedback.distance_remaining());
    }
    const bool terminal = IsNavTerminalStatus(feedback.status());
    *response.mutable_status() =
        terminal && feedback.status() != task_proto::NAV_STATUS_SUCCEEDED
            ? ErrorStatus(StatusCode::TASK_FAILED, "")
            : OkStatus();
    return response;
}

NavigatorTraits::Response NavigatorTraits::MakeResponse(
    const Request& request, bool success, bool final,
    const std::string& message) {
    Response response;
    response.set_goal_id(request.goal_id());
    response.set_number_of_waypoints(request.waypoints_size());
    if (success) {
        response.set_state(final ? nav_rpc::NAVIGATION_STATE_CANCELLED
                                 : nav_rpc::NAVIGATION_STATE_RUNNING);
        *response.mutable_status() = OkStatus(message);
    } else {
        response.set_state(nav_rpc::NAVIGATION_STATE_FAILED);
        *response.mutable_status() =
            ErrorStatus(StatusCode::TASK_FAILED, message);
    }
    return response;
}

bool NavigatorTraits::IsTerminal(const Feedback& feedback) {
    return IsNavTerminalStatus(feedback.status());
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
