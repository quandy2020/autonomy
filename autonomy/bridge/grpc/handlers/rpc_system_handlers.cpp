/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_system_handlers.cpp
 * @brief Implementation of SystemService RpcHandler OnRequest bodies.
 */

#include "autonomy/bridge/grpc/handlers/rpc_system_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/util.hpp"
#include "autonomy/bridge/grpc/profile.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include <chrono>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {
namespace {

::automsgs::rpcs::system::GoalKind ToGoalKind(TaskType type) {
    switch (type) {
        case TASK_TYPE_NAVIGATION:
            return ::automsgs::rpcs::system::GOAL_KIND_NAVIGATION;
        case TASK_TYPE_FOLLOW:
            return ::automsgs::rpcs::system::GOAL_KIND_FOLLOW;
        case TASK_TYPE_TELEOP:
            return ::automsgs::rpcs::system::GOAL_KIND_TELEOP;
        case TASK_TYPE_EXPLORATION:
            return ::automsgs::rpcs::system::GOAL_KIND_EXPLORATION;
        case TASK_TYPE_DOCK:
            return ::automsgs::rpcs::system::GOAL_KIND_CHARGE;
        case TASK_TYPE_MAP:
            return ::automsgs::rpcs::system::GOAL_KIND_MAPPING;
        case TASK_TYPE_VOICE:
            return ::automsgs::rpcs::system::GOAL_KIND_VOICE;
        default:
            return ::automsgs::rpcs::system::GOAL_KIND_UNKNOWN;
    }
}

}  // namespace

void RpcSystemHeartbeatHandler::OnRequest(
    const ::automsgs::rpcs::system::HeartbeatRequest& request) {
    ::automsgs::rpcs::system::HeartbeatResponse response;
    *response.mutable_status() = OkStatus();
    response.set_sequence(request.sequence());
    response.set_robot_time_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    ReplyUnary(this, std::move(response));
}

void RpcSystemGetInfoHandler::OnRequest(
    const ::automsgs::rpcs::system::GetInfoRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetInfoResponse response;
        FillGetInfoResponse(context->identity(), &response);
        return response;
    });
}

void RpcSystemGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::system::GetStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetStatusResponse response;
        *response.mutable_status() = OkStatus();
        if (context->muxer().IsEstop()) {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_ESTOP);
        } else if (context->muxer().HasActive()) {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_BUSY);
        } else {
            response.set_state(::automsgs::rpcs::system::SYSTEM_STATE_IDLE);
        }
        const auto snapshot = context->state_hub().GetSnapshot();
        const float percent = snapshot.battery_percent();
        response.mutable_battery()->set_percentage(
            percent > 1.f ? percent / 100.f : percent);
        *response.mutable_health() = context->system_monitor().GetHealth(false);
        return response;
    });
}

void RpcSystemGetHealthHandler::OnRequest(
    const ::automsgs::rpcs::system::GetHealthRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::GetHealthResponse response;
        *response.mutable_status() = OkStatus();
        *response.mutable_health() = context->system_monitor().GetHealth(true);
        return response;
    });
}

void RpcSystemGetRobotFullInfoHandler::OnRequest(
    const ::automsgs::rpcs::system::GetRobotFullInfoRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return BuildRpcRobotFullInfo(*context);
    });
}

void RpcSystemGetCapabilitiesHandler::OnRequest(
    const ::automsgs::rpcs::system::GetCapabilitiesRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::Capabilities response;
        FillRpcCapabilities(context->capabilities(), &response);
        return response;
    });
}

void RpcSystemEmergencyStopHandler::OnRequest(
    const ::automsgs::rpcs::system::EmergencyStopRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->EmergencyStop(true);
        return OkStatus("estop engaged");
    });
}

void RpcSystemClearEmergencyStopHandler::OnRequest(
    const ::automsgs::rpcs::system::ClearEmergencyStopRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->EmergencyStop(false);
        return OkStatus("estop cleared");
    });
}

void RpcSystemCancelAllGoalsHandler::OnRequest(
    const ::automsgs::rpcs::system::CancelAllGoalsRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->CancelAllTasks();
        return OkStatus("cancelled");
    });
}

void RpcSystemGetActiveGoalHandler::OnRequest(
    const ::automsgs::rpcs::system::GetActiveGoalRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        ::automsgs::rpcs::system::ActiveGoal response;
        *response.mutable_status() = OkStatus();
        const auto snapshot = context->muxer().GetSnapshot();
        response.set_kind(ToGoalKind(snapshot.type));
        response.set_goal_id(snapshot.cmd_id);
        return response;
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
