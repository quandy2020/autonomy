/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_charge_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcChargeReturnHandler::OnRequest(
    const ::automsgs::rpcs::charge::ReturnRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    const auto bridge_request = ToBridgeDockReturn(request);
    const std::string goal_id = request.goal_id();
    context->dock().HandleCommand(
        bridge_request,
        StreamMappedUntilAckFinal(this, [goal_id](const auto& response) {
            return std::make_unique<::automsgs::rpcs::charge::ChargeResponse>(
                ToRpcChargeResponse(response, goal_id));
        }));
}

void RpcChargeLeaveHandler::OnRequest(
    const ::automsgs::rpcs::charge::LeaveRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    const auto bridge_request = ToBridgeDockLeave(request);
    const std::string goal_id = request.goal_id();
    context->dock().HandleCommand(
        bridge_request,
        StreamMappedUntilAckFinal(this, [goal_id](const auto& response) {
            return std::make_unique<::automsgs::rpcs::charge::ChargeResponse>(
                ToRpcChargeResponse(response, goal_id));
        }));
}

void RpcChargeCancelHandler::OnRequest(
    const ::automsgs::rpcs::charge::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->dock().CancelActiveSession();
        return MakeOkStatus("cancelled");
    });
}

void RpcChargePauseHandler::OnRequest(
    const ::automsgs::rpcs::charge::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::DockCommandRequest request;
        request.set_command(proto::DOCK_CMD_PAUSE);
        context->dock().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("paused");
    });
}

void RpcChargeResumeHandler::OnRequest(
    const ::automsgs::rpcs::charge::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::DockCommandRequest request;
        request.set_command(proto::DOCK_CMD_RESUME);
        context->dock().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("resumed");
    });
}

void RpcChargeGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::charge::GetStatusRequest&) {
    ::automsgs::rpcs::charge::ChargeResponse response;
    *response.mutable_status() = MakeOkStatus();
    response.set_state(::automsgs::rpcs::charge::CHARGE_STATE_IDLE);
    response.set_active(false);
    ReplyUnary(this, std::move(response));
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

