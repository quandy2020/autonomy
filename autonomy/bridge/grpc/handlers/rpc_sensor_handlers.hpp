/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>
#include "autonomy/common/async_grpc/rpc_handler.h"
#include <automsgs/rpcs/common.pb.h>
#include <automsgs/rpcs/sensor.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

DEFINE_HANDLER_SIGNATURE(
    RpcSensorListSignature, ::automsgs::rpcs::sensor::ListSensorsRequest,
    ::automsgs::rpcs::sensor::ListSensorsResponse,
    "/automsgs.rpcs.sensor.SensorService/ListSensors")

class RpcSensorListHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSensorListSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::sensor::ListSensorsRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorGetSampleSignature, ::automsgs::rpcs::sensor::GetSampleRequest,
    ::automsgs::rpcs::sensor::GetSampleResponse,
    "/automsgs.rpcs.sensor.SensorService/GetSample")

class RpcSensorGetSampleHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSensorGetSampleSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::sensor::GetSampleRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorGetParametersSignature,
    ::automsgs::rpcs::sensor::GetParametersRequest,
    ::automsgs::rpcs::sensor::GetParametersResponse,
    "/automsgs.rpcs.sensor.SensorService/GetParameters")

class RpcSensorGetParametersHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSensorGetParametersSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::sensor::GetParametersRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorSetParametersSignature,
    ::automsgs::rpcs::sensor::SetParametersRequest,
    ::automsgs::rpcs::sensor::SetParametersResponse,
    "/automsgs.rpcs.sensor.SensorService/SetParameters")

class RpcSensorSetParametersHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSensorSetParametersSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::sensor::SetParametersRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorSaveParametersSignature,
    ::automsgs::rpcs::sensor::SaveParametersRequest,
    ::automsgs::rpcs::sensor::SaveParametersResponse,
    "/automsgs.rpcs.sensor.SensorService/SaveParameters")

class RpcSensorSaveParametersHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSensorSaveParametersSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::sensor::SaveParametersRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorLoadParametersSignature,
    ::automsgs::rpcs::sensor::LoadParametersRequest,
    ::automsgs::rpcs::sensor::LoadParametersResponse,
    "/automsgs.rpcs.sensor.SensorService/LoadParameters")

class RpcSensorLoadParametersHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSensorLoadParametersSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::sensor::LoadParametersRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorRecordSignature, ::automsgs::rpcs::sensor::RecordRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::sensor::RecordResponse>,
    "/automsgs.rpcs.sensor.SensorService/Record")

class RpcSensorRecordHandler
    : public autonomy::common::async_grpc::RpcHandler<RpcSensorRecordSignature> {
public:
    void OnRequest(const ::automsgs::rpcs::sensor::RecordRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorCancelRecordSignature,
    ::automsgs::rpcs::sensor::CancelRecordRequest,
    ::automsgs::rpcs::sensor::CancelRecordResponse,
    "/automsgs.rpcs.sensor.SensorService/CancelRecord")

class RpcSensorCancelRecordHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSensorCancelRecordSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::sensor::CancelRecordRequest& request) override;
};

DEFINE_HANDLER_SIGNATURE(
    RpcSensorGetRecordStatusSignature,
    ::automsgs::rpcs::sensor::GetRecordStatusRequest,
    ::automsgs::rpcs::sensor::GetRecordStatusResponse,
    "/automsgs.rpcs.sensor.SensorService/GetRecordStatus")

class RpcSensorGetRecordStatusHandler
    : public autonomy::common::async_grpc::RpcHandler<
          RpcSensorGetRecordStatusSignature> {
public:
    void OnRequest(
        const ::automsgs::rpcs::sensor::GetRecordStatusRequest& request) override;
};

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

