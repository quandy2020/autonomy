/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_system_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <chrono>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcHeartbeatHandler::OnRequest(
    const ::automsgs::rpcs::system::HeartbeatRequest& request) {
    ::automsgs::rpcs::system::HeartbeatResponse response;
    *response.mutable_status() = MakeOkStatus();
    response.set_sequence(request.sequence());
    response.set_robot_time_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    ReplyUnary(this, std::move(response));
}

void RpcSystemGetInfoHandler::OnRequest(
    const ::automsgs::rpcs::system::GetInfoRequest&) {
    ::automsgs::rpcs::system::GetInfoResponse response;
    *response.mutable_status() = MakeOkStatus();
    response.set_model("autonomy");
    response.set_software_version("autonomy");
    response.set_autonomy_version("autonomy");
    char host[256] = {};
    if (gethostname(host, sizeof(host) - 1) == 0) {
        response.set_hostname(host);
    }
    ReplyUnary(this, std::move(response));
}

void RpcSystemGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::system::GetStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetStatusResponse response;
        *response.mutable_status() = MakeOkStatus();
        if (context->muxer().CheckEstopActive()) {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_ESTOP);
        } else if (context->muxer().CheckHasActive()) {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_BUSY);
        } else {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_IDLE);
        }
        const auto snapshot = context->state_hub().GetSnapshot();
        const float percent = snapshot.battery_percent();
        response.mutable_battery()->set_percentage(percent > 1.f ? percent / 100.f : percent);
        *response.mutable_health() = context->system_monitor().GetHealth(false);
        return response;
    });
}

void RpcSystemGetHealthHandler::OnRequest(
    const ::automsgs::rpcs::system::GetHealthRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetHealthResponse response;
        *response.mutable_status() = MakeOkStatus();
        *response.mutable_health() = context->system_monitor().GetHealth(true);
        return response;
    });
}

void RpcSystemCapabilitiesHandler::OnRequest(
    const ::automsgs::rpcs::system::GetCapabilitiesRequest&) {
    ::automsgs::rpcs::system::Capabilities response;
    *response.mutable_status() = MakeOkStatus();
    response.set_supports_navigation(true);
    response.set_supports_follow(true);
    response.set_supports_charge(true);
    response.set_supports_mapping(true);
    response.set_supports_localization(true);
    response.set_supports_teleop(true);
    response.set_supports_exploration(true);
    response.set_supports_sensor_record(true);
    response.set_supports_system_monitor(true);
    response.set_bridge_version("autonomy.bridge");
    response.set_autonomy_version("autonomy");
    ReplyUnary(this, std::move(response));
}

void RpcSystemEstopHandler::OnRequest(
    const ::automsgs::rpcs::system::EmergencyStopRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->EmergencyStop(true);
        return MakeOkStatus("estop engaged");
    });
}

void RpcSystemClearEstopHandler::OnRequest(
    const ::automsgs::rpcs::system::ClearEmergencyStopRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->EmergencyStop(false);
        return MakeOkStatus("estop cleared");
    });
}

void RpcSystemCancelAllHandler::OnRequest(
    const ::automsgs::rpcs::system::CancelAllGoalsRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->CancelAllTasks();
        return MakeOkStatus("cancelled");
    });
}

using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

::automsgs::rpcs::system::GoalKind ToGoalKind(proto::TaskType type) {
    switch (type) {
        case proto::TASK_TYPE_NAVIGATION:
            return ::automsgs::rpcs::system::GOAL_KIND_NAVIGATION;
        case proto::TASK_TYPE_FOLLOW:
            return ::automsgs::rpcs::system::GOAL_KIND_FOLLOW;
        case proto::TASK_TYPE_TELEOP:
            return ::automsgs::rpcs::system::GOAL_KIND_TELEOP;
        case proto::TASK_TYPE_EXPLORATION:
            return ::automsgs::rpcs::system::GOAL_KIND_EXPLORATION;
        case proto::TASK_TYPE_DOCK:
            return ::automsgs::rpcs::system::GOAL_KIND_CHARGE;
        case proto::TASK_TYPE_MAP:
            return ::automsgs::rpcs::system::GOAL_KIND_MAPPING;
        default:
            return ::automsgs::rpcs::system::GOAL_KIND_UNKNOWN;
    }
}

void RpcSystemActiveGoalHandler::OnRequest(
    const ::automsgs::rpcs::system::GetActiveGoalRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::ActiveGoal response;
        *response.mutable_status() = MakeOkStatus();
        const auto snapshot = context->muxer().GetSnapshot();
        response.set_kind(ToGoalKind(snapshot.type()));
        response.set_goal_id(snapshot.cmd_id());
        return response;
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

