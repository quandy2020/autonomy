/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_localization_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcLocGetPoseHandler::OnRequest(
    const ::automsgs::rpcs::localization::GetPoseRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->localization().GetPose(request);
    });
}

void RpcLocGetStatusHandler::OnRequest(
    const ::automsgs::rpcs::localization::GetStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->localization().GetStatus();
    });
}

void RpcLocSetInitialPoseHandler::OnRequest(
    const ::automsgs::rpcs::localization::SetInitialPoseRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->localization().SetInitialPose(request);
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

