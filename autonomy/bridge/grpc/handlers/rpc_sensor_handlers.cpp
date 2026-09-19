/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_sensor_handlers.cpp
 * @brief Implementation of RpcSensorRecordHandler OnRequest stream start.
 */

#include "autonomy/bridge/grpc/handlers/rpc_sensor_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/util.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcSensorRecordHandler::OnRequest(
    const ::automsgs::rpcs::sensor::RecordRequest& request) {
    auto* context = RequireContext(this);
    if (!context) {
        return;
    }
    context->sensor().StartRecord(
        request,
        [this](const ::automsgs::rpcs::sensor::RecordResponse& response) {
            const bool final = response.final();
            Send(std::make_unique<::automsgs::rpcs::sensor::RecordResponse>(
                response));
            if (final) {
                Finish(::grpc::Status::OK);
            }
        });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
