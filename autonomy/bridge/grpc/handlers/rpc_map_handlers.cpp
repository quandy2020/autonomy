/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_map_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcStartMappingHandler::OnRequest(
    const ::automsgs::rpcs::mapping::StartMappingRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->map_service().StartMapping(request);
    });
}

void RpcFinishMappingHandler::OnRequest(
    const ::automsgs::rpcs::mapping::FinishMappingRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->map_service().FinishMapping(request);
    });
}

void RpcCancelMappingHandler::OnRequest(
    const ::automsgs::rpcs::mapping::CancelMappingRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->map_service().CancelMapping(request);
    });
}

void RpcGetMappingStatusHandler::OnRequest(
    const ::automsgs::rpcs::mapping::GetMappingStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->map_service().GetMappingStatus();
    });
}

void RpcListMapsHandler::OnRequest(
    const ::automsgs::rpcs::mapping::ListMapsRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->map_service().ListMaps();
    });
}

void RpcGetMapHandler::OnRequest(
    const ::automsgs::rpcs::mapping::GetMapRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->map_service().GetMap(request);
    });
}

void RpcGetMapMetadataHandler::OnRequest(
    const ::automsgs::rpcs::mapping::GetMapMetadataRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->map_service().GetMapMetadata(request);
    });
}

void RpcSaveMapHandler::OnRequest(
    const ::automsgs::rpcs::mapping::SaveMapRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->map_service().SaveMap(request);
    });
}

void RpcDeleteMapHandler::OnRequest(
    const ::automsgs::rpcs::mapping::DeleteMapRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->map_service().DeleteMap(request);
    });
}

void RpcSetCurrentMapHandler::OnRequest(
    const ::automsgs::rpcs::mapping::SetCurrentMapRequest& request) {
    ReplyStatusWithContext(this, [&](auto* context) {
        return context->map_service().SetCurrentMap(request);
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

