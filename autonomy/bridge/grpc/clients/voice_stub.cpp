/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/voice_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>
#include <automsgs/task/charging.pb.h>
#include <automsgs/task/exploration.pb.h>
#include <automsgs/task/navigation.pb.h>
#include <automsgs/task/tracker.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;
namespace voice_rpc = ::automsgs::rpcs::voice;
namespace follow_rpc = ::automsgs::rpcs::follow;
namespace charge_rpc = ::automsgs::rpcs::charge;
namespace explore_rpc = ::automsgs::rpcs::exploration;
namespace nav_rpc = ::automsgs::rpcs::navigation;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

task_proto::VoiceIntent ToTaskIntent(voice_rpc::VoiceIntent intent) {
    switch (intent) {
        case voice_rpc::VOICE_INTENT_NAVIGATE:
            return task_proto::VOICE_INTENT_NAVIGATE;
        case voice_rpc::VOICE_INTENT_FOLLOW:
            return task_proto::VOICE_INTENT_FOLLOW;
        case voice_rpc::VOICE_INTENT_DOCK:
            return task_proto::VOICE_INTENT_DOCK;
        case voice_rpc::VOICE_INTENT_UNDOCK:
            return task_proto::VOICE_INTENT_UNDOCK;
        case voice_rpc::VOICE_INTENT_EXPLORE:
            return task_proto::VOICE_INTENT_EXPLORE;
        case voice_rpc::VOICE_INTENT_STOP:
            return task_proto::VOICE_INTENT_STOP;
        case voice_rpc::VOICE_INTENT_CANCEL_ALL:
            return task_proto::VOICE_INTENT_CANCEL_ALL;
        default:
            return task_proto::VOICE_INTENT_UNKNOWN;
    }
}

voice_rpc::VoiceState ToRpcState(task_proto::VoiceStatus status) {
    switch (status) {
        case task_proto::VOICE_STATUS_IDLE:
            return voice_rpc::VOICE_STATE_IDLE;
        case task_proto::VOICE_STATUS_DISPATCHING:
            return voice_rpc::VOICE_STATE_DISPATCHING;
        case task_proto::VOICE_STATUS_RUNNING:
            return voice_rpc::VOICE_STATE_RUNNING;
        case task_proto::VOICE_STATUS_SUCCEEDED:
            return voice_rpc::VOICE_STATE_SUCCEEDED;
        case task_proto::VOICE_STATUS_FAILED:
            return voice_rpc::VOICE_STATE_FAILED;
        case task_proto::VOICE_STATUS_CANCELED:
            return voice_rpc::VOICE_STATE_CANCELLED;
        default:
            return voice_rpc::VOICE_STATE_UNKNOWN;
    }
}

bool IsVoiceTerminalStatus(task_proto::VoiceStatus status) {
    return status == task_proto::VOICE_STATUS_SUCCEEDED ||
           status == task_proto::VOICE_STATUS_FAILED ||
           status == task_proto::VOICE_STATUS_CANCELED;
}

task_proto::NavigationGoal ToNavigationGoal(
    const nav_rpc::NavigateRequest& request) {
    task_proto::NavigationGoal goal;
    goal.set_command(task_proto::NAV_CMD_START);
    goal.set_mode(request.waypoints_size() > 1
                      ? task_proto::NAV_MODE_THROUGH_POSES
                      : task_proto::NAV_MODE_SINGLE_POSE);
    for (const auto& waypoint : request.waypoints()) {
        *goal.add_goals() = waypoint;
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NAVIGATION);
    return goal;
}

task_proto::TrackerGoal ToTrackerGoal(const follow_rpc::FollowRequest& request) {
    task_proto::TrackerGoal goal;
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

task_proto::ChargingGoal ToChargeGoal(const charge_rpc::ReturnRequest& request) {
    task_proto::ChargingGoal goal;
    goal.set_command(task_proto::DOCK_CMD_START);
    if (!request.station_id().empty()) {
        goal.set_dock_station_id(request.station_id());
    }
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK);
    return goal;
}

task_proto::ChargingGoal ToUndockGoal(const charge_rpc::LeaveRequest& request) {
    task_proto::ChargingGoal goal;
    goal.set_command(task_proto::DOCK_CMD_UNDOCK);
    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK);
    return goal;
}

task_proto::ExplorationGoal ToExploreGoal(
    const explore_rpc::ExploreRequest& request) {
    task_proto::ExplorationGoal goal;
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

}  // namespace

VoiceTraits::Goal VoiceTraits::ConvertToGoal(const Request& request) {
    Goal goal;
    const auto intent = ToTaskIntent(request.intent());
    goal.set_intent(intent);
    goal.set_transcript(request.transcript());

    if (intent == task_proto::VOICE_INTENT_STOP ||
        intent == task_proto::VOICE_INTENT_CANCEL_ALL) {
        goal.set_command(task_proto::VOICE_CMD_STOP_ALL);
    } else {
        goal.set_command(task_proto::VOICE_CMD_START);
    }

    switch (intent) {
        case task_proto::VOICE_INTENT_NAVIGATE:
            if (request.has_navigate()) {
                *goal.mutable_navigate() = ToNavigationGoal(request.navigate());
            }
            break;
        case task_proto::VOICE_INTENT_FOLLOW:
            if (request.has_follow()) {
                *goal.mutable_follow() = ToTrackerGoal(request.follow());
            }
            break;
        case task_proto::VOICE_INTENT_DOCK:
            if (request.has_dock()) {
                *goal.mutable_charge() = ToChargeGoal(request.dock());
            }
            break;
        case task_proto::VOICE_INTENT_UNDOCK:
            if (request.has_undock()) {
                *goal.mutable_charge() = ToUndockGoal(request.undock());
            }
            break;
        case task_proto::VOICE_INTENT_EXPLORE:
            if (request.has_explore()) {
                *goal.mutable_explore() = ToExploreGoal(request.explore());
            }
            break;
        default:
            break;
    }

    SetTaskHeader(&goal, request.goal_id(),
                  ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_VOICE);
    return goal;
}

VoiceTraits::Response VoiceTraits::ConvertFromFeedback(
    const Feedback& feedback, const Request& last) {
    Response response;
    response.set_goal_id(last.goal_id());
    response.set_intent(last.intent());
    response.set_state(ToRpcState(feedback.status()));
    response.set_detail(feedback.detail());
    const bool terminal = IsVoiceTerminalStatus(feedback.status());
    response.set_active(!terminal);
    const bool ok =
        !terminal || feedback.status() == task_proto::VOICE_STATUS_SUCCEEDED;
    *response.mutable_status() =
        ok ? OkStatus(feedback.detail())
           : ErrorStatus(StatusCode::TASK_FAILED, feedback.detail());
    return response;
}

VoiceTraits::Response VoiceTraits::MakeResponse(const Request& request,
                                                bool success, bool final,
                                                const std::string& message) {
    Response response;
    response.set_goal_id(request.goal_id());
    response.set_intent(request.intent());
    response.set_state(success
                           ? (final ? voice_rpc::VOICE_STATE_CANCELLED
                                    : voice_rpc::VOICE_STATE_DISPATCHING)
                           : voice_rpc::VOICE_STATE_FAILED);
    response.set_detail(message);
    response.set_active(success && !final);
    *response.mutable_status() =
        success ? OkStatus(message)
                : ErrorStatus(StatusCode::TASK_FAILED, message);
    return response;
}

bool VoiceTraits::IsTerminal(const Feedback& feedback) {
    return IsVoiceTerminalStatus(feedback.status());
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
