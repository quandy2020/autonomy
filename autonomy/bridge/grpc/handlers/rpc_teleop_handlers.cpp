/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_teleop_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcTeleopVelocityHandler::OnRequest(
    const ::automsgs::rpcs::teleop::VelocityRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        stream_finished_ = true;
        return;
    }
    goal_id_ = request.goal_id();
    const auto bridge_request = ToBridgeTeleopVelocity(request);
    context->teleop().HandleCommand(
        bridge_request, [this](const proto::TeleopCommandResponse& response) {
            Send(std::make_unique<::automsgs::rpcs::teleop::TeleopResponse>(
                ToRpcTeleopResponse(response, goal_id_)));
            if (response.ack().final() && !stream_finished_) {
                stream_finished_ = true;
                Finish(::grpc::Status::OK);
            }
        });
}

void RpcTeleopVelocityHandler::OnReadsDone() {
    auto* context =
        GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    if (context) {
        context->teleop().ResetSession();
    }
    if (!stream_finished_) {
        stream_finished_ = true;
        Finish(::grpc::Status::OK);
    }
}

void RpcDriveOnHeadingHandler::OnRequest(
    const ::automsgs::rpcs::teleop::DriveOnHeadingRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    context->relative_teleop().DriveOnHeading(
        request, StreamUntil<RpcDriveOnHeadingHandler,
                             ::automsgs::rpcs::teleop::TeleopResponse>(
                     this, [](const auto& response) {
                         return IsTeleopTerminal(response);
                     }));
}

void RpcBackUpHandler::OnRequest(
    const ::automsgs::rpcs::teleop::BackUpRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    context->relative_teleop().BackUp(
        request,
        StreamUntil<RpcBackUpHandler, ::automsgs::rpcs::teleop::TeleopResponse>(
            this, [](const auto& response) {
                return IsTeleopTerminal(response);
            }));
}

void RpcSpinHandler::OnRequest(const ::automsgs::rpcs::teleop::SpinRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    context->relative_teleop().Spin(
        request,
        StreamUntil<RpcSpinHandler, ::automsgs::rpcs::teleop::TeleopResponse>(
            this, [](const auto& response) {
                return IsTeleopTerminal(response);
            }));
}

void RpcTeleopCancelHandler::OnRequest(
    const ::automsgs::rpcs::teleop::GoalRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        context->relative_teleop().Cancel(request.goal_id());
        context->teleop().ResetSession();
        return MakeOkStatus("cancelled");
    });
}

void RpcTeleopPauseHandler::OnRequest(
    const ::automsgs::rpcs::teleop::GoalRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->relative_teleop().Pause(request.goal_id())
                   ? MakeOkStatus("paused")
                   : MakeRpcStatus(
                         ::automsgs::msgs::status_msgs::INVALID_ARGUMENT,
                         "nothing to pause");
    });
}

void RpcTeleopResumeHandler::OnRequest(
    const ::automsgs::rpcs::teleop::GoalRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->relative_teleop().Resume(request.goal_id())
                   ? MakeOkStatus("resumed")
                   : MakeRpcStatus(
                         ::automsgs::msgs::status_msgs::INVALID_ARGUMENT,
                         "nothing to resume");
    });
}

void RpcTeleopGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::teleop::GetStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->relative_teleop().GetSnapshot();
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

