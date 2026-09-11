/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/handlers/rpc_sensor_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/handler_util.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <unistd.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

void RpcSensorListHandler::OnRequest(
    const ::automsgs::rpcs::sensor::ListSensorsRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->sensor().ListSensors();
    });
}

void RpcSensorGetSampleHandler::OnRequest(
    const ::automsgs::rpcs::sensor::GetSampleRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->sensor().GetSample(request);
    });
}

void RpcSensorGetParametersHandler::OnRequest(
    const ::automsgs::rpcs::sensor::GetParametersRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->sensor().GetParameters(request);
    });
}

void RpcSensorSetParametersHandler::OnRequest(
    const ::automsgs::rpcs::sensor::SetParametersRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        return context->sensor().SetParameters(request);
    });
}

void RpcSensorSaveParametersHandler::OnRequest(
    const ::automsgs::rpcs::sensor::SaveParametersRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        ::automsgs::rpcs::sensor::SaveParametersResponse response;
        *response.mutable_status() = context->sensor().SaveParameters(request);
        return response;
    });
}

void RpcSensorLoadParametersHandler::OnRequest(
    const ::automsgs::rpcs::sensor::LoadParametersRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        ::automsgs::rpcs::sensor::LoadParametersResponse response;
        *response.mutable_status() = context->sensor().LoadParameters(request);
        return response;
    });
}

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

void RpcSensorCancelRecordHandler::OnRequest(
    const ::automsgs::rpcs::sensor::CancelRecordRequest& request) {
    ReplyUnaryWithContext(this, [&](auto* context) {
        ::automsgs::rpcs::sensor::CancelRecordResponse response;
        *response.mutable_status() = context->sensor().CancelRecord(request);
        return response;
    });
}

void RpcSensorGetRecordStatusHandler::OnRequest(
    const ::automsgs::rpcs::sensor::GetRecordStatusRequest&) {
    ReplyUnaryWithContext(this, [](auto* context) {
        return context->sensor().GetRecordStatus();
    });
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

