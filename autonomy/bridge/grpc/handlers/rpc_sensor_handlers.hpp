/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_sensor_handlers.hpp
 * @brief SensorService RpcHandlers: list/sample/params + Record stream.
 *
 * @details
 * Forwards to Context::sensor() (SensorStub). Sensor paths are not GoalChannel
 * adapters for task/* topics; they talk to sensor backends / record sessions
 * owned by SensorStub. Record is BRIDGE_DECL with OnRequest implemented in
 * rpc_sensor_handlers.cpp; other RPCs are macro-generated unary wrappers.
 *
 * Generated types (SMART_PTR via macros / BRIDGE_DECL):
 * - RpcSensorListSensorsHandler — BRIDGE_GET ListSensors
 * - RpcSensorGetSampleHandler — BRIDGE_UNARY
 * - RpcSensorGetParametersHandler — BRIDGE_UNARY
 * - RpcSensorSetParametersHandler — BRIDGE_UNARY
 * - RpcSensorSaveParametersHandler — BRIDGE_WRAP
 * - RpcSensorLoadParametersHandler — BRIDGE_WRAP
 * - RpcSensorCancelRecordHandler — BRIDGE_WRAP
 * - RpcSensorRecordHandler — BRIDGE_DECL (stream; body in .cpp)
 * - RpcSensorGetRecordStatusHandler — BRIDGE_GET GetRecordStatus
 *
 * Invariants:
 * - Save/Load/CancelRecord wrap stub Status into response.mutable_status().
 * - Record is BRIDGE_DECL (implemented in rpc_sensor_handlers.cpp).
 * - Handlers never own sensor hardware state; SensorStub does.
 * - Threading: gRPC completion queue; Record stream may relay off-thread.
 *
 * @see SensorStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/sensor.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief SensorService/ListSensors — inventory snapshot. */
BRIDGE_GET(RpcSensorListSensorsHandler,
                       ::automsgs::rpcs::sensor::ListSensorsRequest,
                       ::automsgs::rpcs::sensor::ListSensorsResponse,
                       "/automsgs.rpcs.sensor.SensorService/ListSensors",
                       &Context::sensor, &clients::SensorStub::ListSensors);

/** @brief SensorService/GetSample — one-shot sensor sample. */
BRIDGE_UNARY(RpcSensorGetSampleHandler,
                 ::automsgs::rpcs::sensor::GetSampleRequest,
                 ::automsgs::rpcs::sensor::GetSampleResponse,
                 "/automsgs.rpcs.sensor.SensorService/GetSample", &Context::sensor,
                 &clients::SensorStub::GetSample);

/** @brief SensorService/GetParameters — read sensor parameter bag. */
BRIDGE_UNARY(RpcSensorGetParametersHandler,
                 ::automsgs::rpcs::sensor::GetParametersRequest,
                 ::automsgs::rpcs::sensor::GetParametersResponse,
                 "/automsgs.rpcs.sensor.SensorService/GetParameters",
                 &Context::sensor, &clients::SensorStub::GetParameters);

/** @brief SensorService/SetParameters — write sensor parameter bag. */
BRIDGE_UNARY(RpcSensorSetParametersHandler,
                 ::automsgs::rpcs::sensor::SetParametersRequest,
                 ::automsgs::rpcs::sensor::SetParametersResponse,
                 "/automsgs.rpcs.sensor.SensorService/SetParameters",
                 &Context::sensor, &clients::SensorStub::SetParameters);

/** @brief SensorService/SaveParameters — Status wrapped into response. */
BRIDGE_WRAP(RpcSensorSaveParametersHandler,
                       ::automsgs::rpcs::sensor::SaveParametersRequest,
                       ::automsgs::rpcs::sensor::SaveParametersResponse,
                       "/automsgs.rpcs.sensor.SensorService/SaveParameters",
                       &Context::sensor, &clients::SensorStub::SaveParameters);

/** @brief SensorService/LoadParameters — Status wrapped into response. */
BRIDGE_WRAP(RpcSensorLoadParametersHandler,
                       ::automsgs::rpcs::sensor::LoadParametersRequest,
                       ::automsgs::rpcs::sensor::LoadParametersResponse,
                       "/automsgs.rpcs.sensor.SensorService/LoadParameters",
                       &Context::sensor, &clients::SensorStub::LoadParameters);

/**
 * @brief SensorService/Record — streaming record session (OnRequest in .cpp).
 *
 * @note Implementation lives in rpc_sensor_handlers.cpp; finishes when the
 *       stub reports a terminal RecordResponse.
 */
BRIDGE_DECL(
    RpcSensorRecordHandler, ::automsgs::rpcs::sensor::RecordRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::sensor::RecordResponse>,
    "/automsgs.rpcs.sensor.SensorService/Record");

/** @brief SensorService/CancelRecord — Status wrapped into response. */
BRIDGE_WRAP(RpcSensorCancelRecordHandler,
                       ::automsgs::rpcs::sensor::CancelRecordRequest,
                       ::automsgs::rpcs::sensor::CancelRecordResponse,
                       "/automsgs.rpcs.sensor.SensorService/CancelRecord",
                       &Context::sensor, &clients::SensorStub::CancelRecord);

/** @brief SensorService/GetRecordStatus — active record session snapshot. */
BRIDGE_GET(
    RpcSensorGetRecordStatusHandler,
    ::automsgs::rpcs::sensor::GetRecordStatusRequest,
    ::automsgs::rpcs::sensor::GetRecordStatusResponse,
    "/automsgs.rpcs.sensor.SensorService/GetRecordStatus", &Context::sensor,
    &clients::SensorStub::GetRecordStatus);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
