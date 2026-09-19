/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_system_handlers.hpp
 * @brief SystemService RpcHandlers: heartbeat, info/health, estop, cancel-all.
 *
 * @details
 * All handlers are BRIDGE_DECL (class + OnRequest declaration); bodies live in
 * rpc_system_handlers.cpp. They aggregate Context state: StateHub
 * (@c kRobotStateChannel `/robot_state`), SystemMonitorStub GetHealth,
 * profile.hpp helpers (identity / capabilities / RobotFullInfo), TaskMuxer
 * estop flags, and CancelRegistry::CancelAll for CancelAllGoals / Estop.
 *
 * Declared handlers (each gets AUTONOMY_SMART_PTR_DEFINITIONS via BRIDGE_DECL):
 * - RpcSystemHeartbeatHandler
 * - RpcSystemGetInfoHandler
 * - RpcSystemGetStatusHandler
 * - RpcSystemGetHealthHandler
 * - RpcSystemGetRobotFullInfoHandler
 * - RpcSystemGetCapabilitiesHandler
 * - RpcSystemEmergencyStopHandler
 * - RpcSystemClearEmergencyStopHandler
 * - RpcSystemCancelAllGoalsHandler
 * - RpcSystemGetActiveGoalHandler
 *
 * Invariants:
 * - EmergencyStop / ClearEmergencyStop / CancelAllGoals mutate TaskMuxer +
 *   CancelRegistry.
 * - GetInfo / GetStatus / GetHealth / GetCapabilities / GetActiveGoal are
 *   read-only snapshots.
 * - SMART_PTR aliases are injected by BRIDGE_DECL for each generated class.
 * - Ownership: per-RPC handlers; Context owns stubs / hub / muxer.
 * - Threading: gRPC completion queue; CancelAll must remain idempotent-friendly.
 *
 * @see StateHub
 * @see SystemMonitorStub
 * @see CancelRegistry
 * @see profile.hpp
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief SystemService/Heartbeat — liveness unary (body in .cpp). */
BRIDGE_DECL(RpcSystemHeartbeatHandler, ::automsgs::rpcs::system::HeartbeatRequest,
                ::automsgs::rpcs::system::HeartbeatResponse,
                "/automsgs.rpcs.system.SystemService/Heartbeat");

/** @brief SystemService/GetInfo — identity subset via FillGetInfoResponse. */
BRIDGE_DECL(RpcSystemGetInfoHandler, ::automsgs::rpcs::system::GetInfoRequest,
                ::automsgs::rpcs::system::GetInfoResponse,
                "/automsgs.rpcs.system.SystemService/GetInfo");

/** @brief SystemService/GetStatus — aggregate system status snapshot. */
BRIDGE_DECL(RpcSystemGetStatusHandler,
                ::automsgs::rpcs::system::GetStatusRequest,
                ::automsgs::rpcs::system::GetStatusResponse,
                "/automsgs.rpcs.system.SystemService/GetStatus");

/** @brief SystemService/GetHealth — SystemMonitorStub::GetHealth wrapper. */
BRIDGE_DECL(RpcSystemGetHealthHandler,
                ::automsgs::rpcs::system::GetHealthRequest,
                ::automsgs::rpcs::system::GetHealthResponse,
                "/automsgs.rpcs.system.SystemService/GetHealth");

/** @brief SystemService/GetRobotFullInfo — BuildRpcRobotFullInfo snapshot. */
BRIDGE_DECL(RpcSystemGetRobotFullInfoHandler,
                ::automsgs::rpcs::system::GetRobotFullInfoRequest,
                ::automsgs::rpcs::system::RobotFullInfo,
                "/automsgs.rpcs.system.SystemService/GetRobotFullInfo");

/** @brief SystemService/GetCapabilities — FillRpcCapabilities. */
BRIDGE_DECL(RpcSystemGetCapabilitiesHandler,
                ::automsgs::rpcs::system::GetCapabilitiesRequest,
                ::automsgs::rpcs::system::Capabilities,
                "/automsgs.rpcs.system.SystemService/GetCapabilities");

/**
 * @brief SystemService/EmergencyStop — latch estop + CancelRegistry::CancelAll.
 *
 * @warning Side-effecting; must remain safe when domains are idle.
 */
BRIDGE_DECL(RpcSystemEmergencyStopHandler,
                ::automsgs::rpcs::system::EmergencyStopRequest,
                ::automsgs::rpcs::common::Status,
                "/automsgs.rpcs.system.SystemService/EmergencyStop");

/** @brief SystemService/ClearEmergencyStop — clear muxer estop latch. */
BRIDGE_DECL(RpcSystemClearEmergencyStopHandler,
                ::automsgs::rpcs::system::ClearEmergencyStopRequest,
                ::automsgs::rpcs::common::Status,
                "/automsgs.rpcs.system.SystemService/ClearEmergencyStop");

/**
 * @brief SystemService/CancelAllGoals — CancelRegistry::CancelAll without estop.
 */
BRIDGE_DECL(RpcSystemCancelAllGoalsHandler,
                ::automsgs::rpcs::system::CancelAllGoalsRequest,
                ::automsgs::rpcs::common::Status,
                "/automsgs.rpcs.system.SystemService/CancelAllGoals");

/** @brief SystemService/GetActiveGoal — muxer active-goal snapshot. */
BRIDGE_DECL(RpcSystemGetActiveGoalHandler,
                ::automsgs::rpcs::system::GetActiveGoalRequest,
                ::automsgs::rpcs::system::ActiveGoal,
                "/automsgs.rpcs.system.SystemService/GetActiveGoal");

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
