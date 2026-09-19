/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_teleop_handlers.cpp
 * @brief Implementation of RpcTeleopVelocityHandler OnRequest / OnReadsDone.
 */

#include "autonomy/bridge/grpc/handlers/rpc_teleop_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/util.hpp"

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
    LogIngress("TeleopVelocity", request);
    const bool accepted = context->teleop().HandleVelocity(
        request, [this](const auto& response) {
            Send(std::make_unique<std::decay_t<decltype(response)>>(response));
            if (IsTeleopTerminal(response) && !stream_finished_) {
                stream_finished_ = true;
                Finish(::grpc::Status::OK);
            }
        });
    if (!accepted) {
        AWARN << "Teleop velocity rejected";
    }
}

void RpcTeleopVelocityHandler::OnReadsDone() {
    auto* context = GetUnsynchronizedContext<Context>();
    if (context) {
        context->teleop().ResetSession();
    }
    if (!stream_finished_) {
        stream_finished_ = true;
        Finish(::grpc::Status::OK);
    }
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
