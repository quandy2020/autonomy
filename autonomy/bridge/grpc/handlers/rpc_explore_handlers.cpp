/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_explore_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcExploreHandler::OnRequest(
    const ::automsgs::rpcs::exploration::ExploreRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    const auto bridge_request = ToBridgeExplore(request);
    const std::string goal_id = request.goal_id();
    context->exploration().HandleCommand(
        bridge_request,
        StreamMappedUntilAckFinal(this, [goal_id](const auto& response) {
            return std::make_unique<::automsgs::rpcs::exploration::ExploreResponse>(
                ToRpcExploreResponse(response, goal_id));
        }));
}

void RpcExploreCancelHandler::OnRequest(
    const ::automsgs::rpcs::exploration::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        context->exploration().CancelActiveSession();
        return MakeOkStatus("cancelled");
    });
}

void RpcExplorePauseHandler::OnRequest(
    const ::automsgs::rpcs::exploration::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::ExplorationCommandRequest request;
        request.set_command(proto::EXPLORATION_CMD_PAUSE);
        context->exploration().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("paused");
    });
}

void RpcExploreResumeHandler::OnRequest(
    const ::automsgs::rpcs::exploration::GoalRequest&) {
    ReplyStatusWithContext(this, [](auto* context) {
        proto::ExplorationCommandRequest request;
        request.set_command(proto::EXPLORATION_CMD_RESUME);
        context->exploration().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("resumed");
    });
}

void RpcExploreGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::exploration::GetStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return ToRpcExploreResponse(context->exploration().GetSnapshot(), "");
    });
}

void RpcExploreSetAreaHandler::OnRequest(
    const ::automsgs::rpcs::exploration::SetAreaRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        proto::ExplorationCommandRequest request;
        request.set_command(proto::EXPLORATION_CMD_SET_AREA);
        if (request.has_area()) {
            *request.mutable_area() = request.area();
        }
        context->exploration().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("area updated");
    });
}

void RpcExploreSaveMapHandler::OnRequest(
    const ::automsgs::rpcs::exploration::SaveMapRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        proto::ExplorationCommandRequest request;
        request.set_command(proto::EXPLORATION_CMD_SAVE_MAP);
        request.set_map_name(request.map_name());
        context->exploration().HandleCommand(request, [](const auto&) {});
        return MakeOkStatus("save requested");
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

