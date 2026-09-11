/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/task/charging.pb.h>
#include <automsgs/task/mapping.pb.h>
#include <automsgs/task/tracker.pb.h>
#include <automsgs/rpcs/charge.pb.h>
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/exploration.pb.h>
#include <automsgs/rpcs/follow.pb.h>
#include <automsgs/rpcs/navigation.pb.h>
#include <automsgs/rpcs/teleop.pb.h>
#include <automsgs/rpcs/voice.pb.h>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {

inline ::automsgs::rpcs::common::Status MakeRpcStatus(
    ::automsgs::msgs::status_msgs::StatusCode code,
    const std::string& message = "") {
    ::automsgs::rpcs::common::Status status;
    status.set_code(code);
    if (!message.empty()) {
        status.set_message(message);
    }
    return status;
}

inline ::automsgs::rpcs::common::Status MakeOkStatus(
    const std::string& message = "") {
    return MakeRpcStatus(::automsgs::msgs::status_msgs::OK, message);
}

::autonomy::task::proto::TrackerGoal ToTaskTrackerGoal(
    const proto::FollowCommandRequest& request);

::autonomy::task::proto::ChargingGoal ToTaskChargingGoal(
    const proto::DockCommandRequest& request);

::autonomy::task::proto::MappingGoal ToTaskMappingGoal(
    const proto::MapCommandRequest& request);

proto::NavigationCommandRequest ToBridgeNavigate(
    const ::automsgs::rpcs::navigation::NavigateRequest& request);

proto::FollowCommandRequest ToBridgeFollow(
    const ::automsgs::rpcs::follow::FollowRequest& request);

proto::DockCommandRequest ToBridgeDockReturn(
    const ::automsgs::rpcs::charge::ReturnRequest& request);

proto::DockCommandRequest ToBridgeDockLeave(
    const ::automsgs::rpcs::charge::LeaveRequest& request);

proto::TeleopCommandRequest ToBridgeTeleopVelocity(
    const ::automsgs::rpcs::teleop::VelocityRequest& request);

proto::ExplorationCommandRequest ToBridgeExplore(
    const ::automsgs::rpcs::exploration::ExploreRequest& request);

proto::VoiceCommandRequest ToBridgeVoice(
    const ::automsgs::rpcs::voice::VoiceCommandRequest& request);

::automsgs::rpcs::navigation::NavigateResponse ToRpcNavigateResponse(
    const proto::NavigationCommandResponse& response, const std::string& goal_id);

::automsgs::rpcs::follow::FollowResponse ToRpcFollowResponse(
    const proto::FollowCommandResponse& response, const std::string& goal_id);

::automsgs::rpcs::charge::ChargeResponse ToRpcChargeResponse(
    const proto::DockCommandResponse& response, const std::string& goal_id);

::automsgs::rpcs::teleop::TeleopResponse ToRpcTeleopResponse(
    const proto::TeleopCommandResponse& response, const std::string& goal_id);

::automsgs::rpcs::exploration::ExploreResponse ToRpcExploreResponse(
    const proto::ExplorationCommandResponse& response,
    const std::string& goal_id);

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
