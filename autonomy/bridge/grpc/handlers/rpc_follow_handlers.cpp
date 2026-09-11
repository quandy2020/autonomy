/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_follow_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcFollowHandler::OnRequest(
    const ::automsgs::rpcs::follow::FollowRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    const auto bridge_request = ToBridgeFollow(request);
    const std::string goal_id = request.goal_id();
    context->follow().HandleCommand(
        bridge_request,
        StreamMappedUntilAckFinal(this, [goal_id](const auto& response) {
            return std::make_unique<::automsgs::rpcs::follow::FollowResponse>(
                ToRpcFollowResponse(response, goal_id));
        }));
}

void RpcFollowCancelHandler::OnRequest(
    const ::automsgs::rpcs::follow::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->follow().CancelActiveSession();
        return MakeOkStatus("cancelled");
    });
}

void RpcFollowPauseHandler::OnRequest(
    const ::automsgs::rpcs::follow::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::FollowCommandRequest request;
        request.set_command(proto::FOLLOW_CMD_PAUSE);
        context->follow().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("paused");
    });
}

void RpcFollowResumeHandler::OnRequest(
    const ::automsgs::rpcs::follow::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::FollowCommandRequest request;
        request.set_command(proto::FOLLOW_CMD_RESUME);
        context->follow().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("resumed");
    });
}

void RpcFollowGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::follow::GetStatusRequest&) {
    ::automsgs::rpcs::follow::FollowResponse response;
    *response.mutable_status() = MakeOkStatus();
    response.set_state(::automsgs::rpcs::follow::FOLLOW_STATE_IDLE);
    response.set_active(false);
    ReplyUnary(this, std::move(response));
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

