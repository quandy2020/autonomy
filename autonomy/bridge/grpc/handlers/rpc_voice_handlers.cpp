/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_voice_handlers.hpp"
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

void RpcVoiceExecuteHandler::OnRequest(
    const ::automsgs::rpcs::voice::VoiceCommandRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    const auto bridge_request = ToBridgeVoice(request);
    context->voice().HandleCommand(
        bridge_request, [this](const proto::VoiceCommandResponse& response) {
            auto response =
                std::make_unique<::automsgs::rpcs::voice::VoiceCommandResponse>();
            *response->mutable_status() =
                response.ack().success()
                    ? MakeOkStatus(response.ack().message())
                    : MakeRpcStatus(StatusCode::TASK_FAILED,
                                    response.ack().message());
            response->set_goal_id(response.ack().cmd_id());
            response->set_intent(static_cast<::automsgs::rpcs::voice::VoiceIntent>(
                response.intent()));
            response->set_detail(response.detail());
            response->set_active(!response.ack().final());
            switch (response.status()) {
                case proto::VOICE_STATUS_DISPATCHING:
                    response->set_state(
                        ::automsgs::rpcs::voice::VOICE_STATE_DISPATCHING);
                    break;
                case proto::VOICE_STATUS_RUNNING:
                    response->set_state(::automsgs::rpcs::voice::VOICE_STATE_RUNNING);
                    break;
                case proto::VOICE_STATUS_SUCCEEDED:
                    response->set_state(
                        ::automsgs::rpcs::voice::VOICE_STATE_SUCCEEDED);
                    break;
                case proto::VOICE_STATUS_FAILED:
                    response->set_state(::automsgs::rpcs::voice::VOICE_STATE_FAILED);
                    break;
                case proto::VOICE_STATUS_CANCELED:
                    response->set_state(
                        ::automsgs::rpcs::voice::VOICE_STATE_CANCELLED);
                    break;
                default:
                    response->set_state(::automsgs::rpcs::voice::VOICE_STATE_IDLE);
                    break;
            }
            const bool final = response.ack().final();
            Send(std::move(response));
            if (final) {
                Finish(::grpc::Status::OK);
            }
        });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

