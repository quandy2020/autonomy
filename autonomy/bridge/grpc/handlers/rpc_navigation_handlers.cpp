/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_navigation_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

void RpcNavigateHandler::OnRequest(
    const ::automsgs::rpcs::navigation::NavigateRequest& request) {
    if (request.waypoints_size() == 0) {
        ::automsgs::rpcs::navigation::NavigateResponse response;
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::INVALID_ARGUMENT, "empty waypoints");
        response.set_state(
            ::automsgs::rpcs::navigation::NAVIGATION_STATE_FAILED);
        ReplyUnary(this, std::move(response));
        return;
    }
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    const auto bridge_request = ToBridgeNavigate(request);
    const std::string goal_id = request.goal_id();
    context->navigator().HandleCommand(
        bridge_request,
        StreamMappedUntilAckFinal(this, [goal_id](const auto& response) {
            return std::make_unique<::automsgs::rpcs::navigation::NavigateResponse>(
                ToRpcNavigateResponse(response, goal_id));
        }));
}

void RpcNavCancelHandler::OnRequest(
    const ::automsgs::rpcs::navigation::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::NavigationCommandRequest cancel;
        cancel.set_command(proto::NAV_CMD_CANCEL);
        context->navigator().HandleCommand(cancel, [](const auto&) {});
        return MakeOkStatus("cancelled");
    });
}

void RpcNavPauseHandler::OnRequest(
    const ::automsgs::rpcs::navigation::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::NavigationCommandRequest pause;
        pause.set_command(proto::NAV_CMD_PAUSE);
        context->navigator().HandleCommand(pause, [](const auto&) {});
        return MakeOkStatus("paused");
    });
}

void RpcNavResumeHandler::OnRequest(
    const ::automsgs::rpcs::navigation::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::NavigationCommandRequest resume;
        resume.set_command(proto::NAV_CMD_RESUME);
        context->navigator().HandleCommand(resume, [](const auto&) {});
        return MakeOkStatus("resumed");
    });
}

void RpcNavGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::navigation::GetStatusRequest&) {
    ::automsgs::rpcs::navigation::NavigateResponse response;
    *response.mutable_status() = MakeOkStatus();
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    response.set_state(
        context && context->navigator().CheckNavigating()
            ? ::automsgs::rpcs::navigation::NAVIGATION_STATE_RUNNING
            : ::automsgs::rpcs::navigation::NAVIGATION_STATE_IDLE);
    ReplyUnary(this, std::move(response));
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

