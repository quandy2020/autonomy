/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/rpc_convert.hpp"

#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

namespace task_proto = ::autonomy::task::proto;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

void FillTaskHeader(task_proto::TaskHeader* header, const proto::RequestHeader& request,
                    ::automsgs::msgs::vehicle_msgs::RobotTaskType type) {
    if (request.has_timestamp()) {
        *header->mutable_stamp() = request.timestamp();
    }
    header->set_task_id(request.cmd_id());
    header->set_client_id(request.client_id());
    header->set_task_type(type);
}

proto::RequestHeader MakeHeader(const std::string& goal_id) {
    proto::RequestHeader header;
    header.set_cmd_id(goal_id);
    return header;
}

}  // namespace

task_proto::TrackerGoal ToTaskTrackerGoal(const proto::FollowCommandRequest& request) {
    task_proto::TrackerGoal goal;
    goal.set_command(static_cast<task_proto::TrackerCommand>(request.command()));
    goal.set_mode(static_cast<task_proto::TrackerMode>(request.mode()));
    goal.set_follow_distance(request.follow_distance());
    goal.set_follow_angle(request.follow_angle());
    goal.set_session_timeout_sec(request.session_timeout_sec());
    goal.set_reacquire_on_lost(request.reacquire_on_lost());
    if (request.has_header()) {
        FillTaskHeader(goal.mutable_header(), request.header(),
                       ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_FOLLOW);
    }
    if (request.has_target_pose()) {
        *goal.mutable_target_pose() = request.target_pose();
    } else if (request.has_target_id()) {
        goal.set_target_id(request.target_id());
    }
    return goal;
}

task_proto::ChargingGoal ToTaskChargingGoal(const proto::DockCommandRequest& request) {
    task_proto::ChargingGoal goal;
    goal.set_command(static_cast<task_proto::DockCommand>(request.command()));
    goal.set_max_search_radius(request.max_search_radius());
    goal.set_max_retry_count(request.max_retry_count());
    if (request.has_header()) {
        FillTaskHeader(goal.mutable_header(), request.header(),
                       ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_DOCK);
    }
    if (request.has_dock_station_id()) {
        goal.set_dock_station_id(request.dock_station_id());
    } else if (request.has_dock_pose()) {
        *goal.mutable_dock_pose() = request.dock_pose();
    }
    return goal;
}

task_proto::MappingGoal ToTaskMappingGoal(const proto::MapCommandRequest& request) {
    task_proto::MappingGoal goal;
    goal.set_command(static_cast<task_proto::MapCommand>(request.command()));
    if (request.has_header()) {
        FillTaskHeader(goal.mutable_header(), request.header(),
                       ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_MAP);
    }
    if (request.has_map_name()) {
        goal.set_map_name(request.map_name());
    } else if (request.has_initial_pose()) {
        *goal.mutable_initial_pose() = request.initial_pose();
    } else if (request.has_keep_out_zone()) {
        *goal.mutable_keep_out_zone() = request.keep_out_zone();
    } else if (request.has_zone_id()) {
        goal.set_zone_id(request.zone_id());
    }
    return goal;
}

proto::NavigationCommandRequest ToBridgeNavigate(
    const ::automsgs::rpcs::navigation::NavigateRequest& request) {
    proto::NavigationCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    bridge_request.set_command(proto::NAV_CMD_START);
    bridge_request.set_mode(request.waypoints_size() > 1 ? proto::NAV_MODE_THROUGH_POSES
                                              : proto::NAV_MODE_SINGLE_POSE);
    for (const auto& pose : request.waypoints()) {
        *bridge_request.add_goals() = pose;
    }
    if (request.has_options() && request.options().has_timeout_seconds()) {
        bridge_request.set_timeout_sec(request.options().timeout_seconds());
    }
    return bridge_request;
}

proto::FollowCommandRequest ToBridgeFollow(
    const ::automsgs::rpcs::follow::FollowRequest& request) {
    proto::FollowCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    bridge_request.set_command(proto::FOLLOW_CMD_START);
    switch (request.target_type()) {
        case ::automsgs::rpcs::follow::FOLLOW_TARGET_TYPE_PERSON:
            bridge_request.set_mode(proto::FOLLOW_MODE_PERSON);
            break;
        case ::automsgs::rpcs::follow::FOLLOW_TARGET_TYPE_CUSTOM:
            bridge_request.set_mode(proto::FOLLOW_MODE_TARGET_ID);
            break;
        default:
            bridge_request.set_mode(request.target_id().empty()
                             ? proto::FOLLOW_MODE_PERSON
                             : proto::FOLLOW_MODE_TARGET_ID);
            break;
    }
    if (!request.target_id().empty()) {
        bridge_request.set_target_id(request.target_id());
    }
    if (request.has_hint_pose()) {
        auto* pose = bridge_request.mutable_target_pose();
        *pose->mutable_pose() = request.hint_pose();
        if (request.has_header()) {
            *pose->mutable_header() = request.header();
        }
    }
    if (request.has_options() &&
        request.options().has_desired_distance_m()) {
        bridge_request.set_follow_distance(request.options().desired_distance_m());
    }
    bridge_request.set_reacquire_on_lost(true);
    return bridge_request;
}

proto::DockCommandRequest ToBridgeDockReturn(
    const ::automsgs::rpcs::charge::ReturnRequest& request) {
    proto::DockCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    bridge_request.set_command(proto::DOCK_CMD_START);
    if (!request.station_id().empty()) {
        bridge_request.set_dock_station_id(request.station_id());
    }
    return bridge_request;
}

proto::DockCommandRequest ToBridgeDockLeave(
    const ::automsgs::rpcs::charge::LeaveRequest& request) {
    proto::DockCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    bridge_request.set_command(proto::DOCK_CMD_UNDOCK);
    return bridge_request;
}

proto::TeleopCommandRequest ToBridgeTeleopVelocity(
    const ::automsgs::rpcs::teleop::VelocityRequest& request) {
    proto::TeleopCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    switch (request.command()) {
        case ::automsgs::rpcs::teleop::VELOCITY_COMMAND_START:
            bridge_request.set_command(proto::TELEOP_CMD_START);
            break;
        case ::automsgs::rpcs::teleop::VELOCITY_COMMAND_STOP:
            bridge_request.set_command(proto::TELEOP_CMD_STOP);
            break;
        case ::automsgs::rpcs::teleop::VELOCITY_COMMAND_TWIST:
            bridge_request.set_command(proto::TELEOP_CMD_VELOCITY);
            break;
        default:
            bridge_request.set_command(proto::TELEOP_CMD_UNSPECIFIED);
            break;
    }
    if (request.has_twist()) {
        *bridge_request.mutable_velocity() = request.twist();
    }
    if (request.has_options()) {
        if (request.options().has_maximum_linear_speed()) {
            bridge_request.set_max_linear_speed(request.options().maximum_linear_speed());
        }
        if (request.options().has_maximum_angular_speed()) {
            bridge_request.set_max_angular_speed(request.options().maximum_angular_speed());
        }
        if (request.options().has_watchdog_timeout_seconds()) {
            bridge_request.set_watchdog_timeout_sec(
                request.options().watchdog_timeout_seconds());
        }
        bridge_request.set_disable_collision_checks(
            request.options().disable_collision_checks());
    }
    return bridge_request;
}

proto::ExplorationCommandRequest ToBridgeExplore(
    const ::automsgs::rpcs::exploration::ExploreRequest& request) {
    proto::ExplorationCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    bridge_request.set_command(proto::EXPLORATION_CMD_START);
    if (request.has_area()) {
        *bridge_request.mutable_area() = request.area();
    }
    bridge_request.set_map_name(request.map_name());
    if (request.has_options()) {
        if (request.options().has_timeout_seconds()) {
            bridge_request.set_timeout_sec(request.options().timeout_seconds());
        }
        if (request.options().has_coverage_target()) {
            bridge_request.set_coverage_target(request.options().coverage_target());
        }
        bridge_request.set_enable_mapping(request.options().enable_mapping());
    } else {
        bridge_request.set_enable_mapping(true);
    }
    return bridge_request;
}

proto::VoiceCommandRequest ToBridgeVoice(
    const ::automsgs::rpcs::voice::VoiceCommandRequest& request) {
    proto::VoiceCommandRequest bridge_request;
    *bridge_request.mutable_header() = MakeHeader(request.goal_id());
    bridge_request.set_transcript(request.transcript());
    bridge_request.set_intent(static_cast<proto::VoiceIntent>(request.intent()));
    if (request.has_navigate()) {
        *bridge_request.mutable_navigate() = ToBridgeNavigate(request.navigate());
    } else if (request.has_follow()) {
        *bridge_request.mutable_follow() = ToBridgeFollow(request.follow());
    } else if (request.has_dock()) {
        *bridge_request.mutable_dock() = ToBridgeDockReturn(request.dock());
    } else if (request.has_undock()) {
        *bridge_request.mutable_dock() = ToBridgeDockLeave(request.undock());
        bridge_request.set_intent(proto::VOICE_INTENT_UNDOCK);
    } else if (request.has_explore()) {
        *bridge_request.mutable_explore() = ToBridgeExplore(request.explore());
    }
    return bridge_request;
}

::automsgs::rpcs::navigation::NavigateResponse ToRpcNavigateResponse(
    const proto::NavigationCommandResponse& response,
    const std::string& goal_id) {
    ::automsgs::rpcs::navigation::NavigateResponse rpc_response;
    rpc_response.set_goal_id(goal_id);
    rpc_response.set_waypoint_index(response.current_waypoint_index());
    rpc_response.set_number_of_waypoints(response.total_waypoints());
    rpc_response.set_remaining_distance_meters(response.distance_remaining());
    if (response.has_current_pose()) {
        *rpc_response.mutable_current_pose() = response.current_pose();
    }
    switch (response.status()) {
        case proto::NAV_STATUS_IDLE:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_IDLE);
            break;
        case proto::NAV_STATUS_PLANNING:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_PLANNING);
            break;
        case proto::NAV_STATUS_NAVIGATING:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_RUNNING);
            break;
        case proto::NAV_STATUS_PAUSED:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_PAUSED);
            break;
        case proto::NAV_STATUS_SUCCEEDED:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_ARRIVED);
            break;
        case proto::NAV_STATUS_FAILED:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_FAILED);
            break;
        case proto::NAV_STATUS_CANCELED:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_CANCELLED);
            break;
        default:
            rpc_response.set_state(
                ::automsgs::rpcs::navigation::NAVIGATION_STATE_UNKNOWN);
            break;
    }
    *rpc_response.mutable_status() = response.ack().success()
                                ? MakeOkStatus(response.ack().message())
                                : MakeRpcStatus(StatusCode::TASK_FAILED,
                                                response.ack().message());
    return rpc_response;
}

::automsgs::rpcs::follow::FollowResponse ToRpcFollowResponse(
    const proto::FollowCommandResponse& response, const std::string& goal_id) {
    ::automsgs::rpcs::follow::FollowResponse rpc_response;
    rpc_response.set_goal_id(goal_id);
    rpc_response.set_distance_m(response.distance_to_target());
    rpc_response.set_message(response.ack().message());
    switch (response.status()) {
        case proto::FOLLOW_STATUS_IDLE:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_IDLE);
            rpc_response.set_active(false);
            break;
        case proto::FOLLOW_STATUS_FOLLOWING:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_FOLLOWING);
            rpc_response.set_active(true);
            break;
        case proto::FOLLOW_STATUS_PAUSED:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_PAUSED);
            rpc_response.set_active(true);
            break;
        case proto::FOLLOW_STATUS_TARGET_LOST:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_LOST_TARGET);
            rpc_response.set_active(true);
            break;
        case proto::FOLLOW_STATUS_FAILED:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_FAILED);
            rpc_response.set_active(false);
            break;
        case proto::FOLLOW_STATUS_CANCELED:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_CANCELLED);
            rpc_response.set_active(false);
            break;
        default:
            rpc_response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_UNKNOWN);
            break;
    }
    *rpc_response.mutable_status() = response.ack().success()
                                ? MakeOkStatus(response.ack().message())
                                : MakeRpcStatus(StatusCode::FOLLOW_BUSY,
                                                response.ack().message());
    return rpc_response;
}

::automsgs::rpcs::charge::ChargeResponse ToRpcChargeResponse(
    const proto::DockCommandResponse& response, const std::string& goal_id) {
    ::automsgs::rpcs::charge::ChargeResponse rpc_response;
    rpc_response.set_goal_id(goal_id);
    rpc_response.set_station_id(response.dock_station_id());
    rpc_response.set_battery_pct(response.battery_percent());
    rpc_response.set_message(response.ack().message());
    switch (response.status()) {
        case proto::DOCK_STATUS_IDLE:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_IDLE);
            rpc_response.set_active(false);
            break;
        case proto::DOCK_STATUS_SEARCHING:
        case proto::DOCK_STATUS_APPROACHING:
        case proto::DOCK_STATUS_DOCKING:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_RETURNING);
            rpc_response.set_active(true);
            break;
        case proto::DOCK_STATUS_CHARGING:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_CHARGING);
            rpc_response.set_active(true);
            break;
        case proto::DOCK_STATUS_UNDOCKING:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_LEAVING);
            rpc_response.set_active(true);
            break;
        case proto::DOCK_STATUS_SUCCEEDED:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_FULL);
            rpc_response.set_active(false);
            break;
        case proto::DOCK_STATUS_FAILED:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_FAILED);
            rpc_response.set_active(false);
            break;
        case proto::DOCK_STATUS_CANCELED:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_CANCELLED);
            rpc_response.set_active(false);
            break;
        default:
            rpc_response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_UNKNOWN);
            break;
    }
    *rpc_response.mutable_status() = response.ack().success()
                                ? MakeOkStatus(response.ack().message())
                                : MakeRpcStatus(StatusCode::CHARGING_BUSY,
                                                response.ack().message());
    return rpc_response;
}

::automsgs::rpcs::teleop::TeleopResponse ToRpcTeleopResponse(
    const proto::TeleopCommandResponse& response, const std::string& goal_id) {
    ::automsgs::rpcs::teleop::TeleopResponse rpc_response;
    rpc_response.set_goal_id(goal_id);
    rpc_response.set_detail(response.ack().message());
    switch (response.status()) {
        case proto::TELEOP_STATUS_IDLE:
            rpc_response.set_state(::automsgs::rpcs::teleop::TELEOP_STATE_IDLE);
            break;
        case proto::TELEOP_STATUS_ACTIVE:
            rpc_response.set_state(::automsgs::rpcs::teleop::TELEOP_STATE_ACTIVE);
            break;
        case proto::TELEOP_STATUS_TIMEOUT:
            rpc_response.set_state(::automsgs::rpcs::teleop::TELEOP_STATE_TIMEOUT);
            break;
        case proto::TELEOP_STATUS_REJECTED:
            rpc_response.set_state(::automsgs::rpcs::teleop::TELEOP_STATE_REJECTED);
            break;
        default:
            rpc_response.set_state(::automsgs::rpcs::teleop::TELEOP_STATE_UNKNOWN);
            break;
    }
    *rpc_response.mutable_status() = response.ack().success()
                                ? MakeOkStatus(response.ack().message())
                                : MakeRpcStatus(StatusCode::TELEOP_BUSY,
                                                response.ack().message());
    return rpc_response;
}

::automsgs::rpcs::exploration::ExploreResponse ToRpcExploreResponse(
    const proto::ExplorationCommandResponse& response,
    const std::string& goal_id) {
    ::automsgs::rpcs::exploration::ExploreResponse rpc_response;
    rpc_response.set_goal_id(goal_id);
    rpc_response.set_map_name(response.map_name());
    rpc_response.set_frontier_count(response.frontier_count());
    rpc_response.set_progress(response.progress());
    rpc_response.set_detail(response.ack().message());
    if (response.has_current_pose()) {
        *rpc_response.mutable_current_pose() = response.current_pose();
    }
    switch (response.status()) {
        case proto::EXPLORATION_STATUS_IDLE:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_IDLE);
            rpc_response.set_active(false);
            break;
        case proto::EXPLORATION_STATUS_PLANNING:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_PLANNING);
            rpc_response.set_active(true);
            break;
        case proto::EXPLORATION_STATUS_EXPLORING:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_EXPLORING);
            rpc_response.set_active(true);
            break;
        case proto::EXPLORATION_STATUS_PAUSED:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_PAUSED);
            rpc_response.set_active(true);
            break;
        case proto::EXPLORATION_STATUS_SUCCEEDED:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_COMPLETED);
            rpc_response.set_active(false);
            break;
        case proto::EXPLORATION_STATUS_FAILED:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_FAILED);
            rpc_response.set_active(false);
            break;
        case proto::EXPLORATION_STATUS_CANCELED:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_CANCELLED);
            rpc_response.set_active(false);
            break;
        default:
            rpc_response.set_state(
                ::automsgs::rpcs::exploration::EXPLORATION_STATE_UNKNOWN);
            break;
    }
    *rpc_response.mutable_status() =
        response.ack().success()
            ? MakeOkStatus(response.ack().message())
            : MakeRpcStatus(StatusCode::EXPLORATION_BUSY,
                            response.ack().message());
    return rpc_response;
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
