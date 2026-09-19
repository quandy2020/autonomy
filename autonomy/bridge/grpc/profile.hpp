/*
 * Copyright 2026 The Openbot Authors
 *
 * Aggregate robot profile for client / cloud onboarding (Unary full snapshot).
 */

/**
 * @file profile.hpp
 * @brief Fill / build SystemService identity, capabilities, and RobotFullInfo.
 *
 * @details
 * Pure free-function helpers (no class types): map bridge.proto options and
 * live Context state into automsgs.rpcs.system messages for GetRobotFullInfo /
 * GetInfo / GetCapabilities. Callers pass output pointers that must be
 * non-null; helpers clear / set fields on the provided messages.
 *
 * Data sources:
 * - proto::RobotIdentityOptions / CapabilitiesOptions from bridge options
 * - Context (identity, capabilities, StateHub, muxer readiness) for full info
 * - StateHub reads @c kRobotStateChannel (`/robot_state`) via GetSnapshot
 *
 * Invariants:
 * - No GoalChannel publishes; read-only aggregation for System RPCs.
 * - Capability mapping must stay centralized here (no forked defaults in
 *   handlers).
 * - BuildRpcRobotFullInfo returns by value for Unary handler convenience.
 *
 * Ownership: none (free functions); Context / options owned by callers.
 * Threading: safe on gRPC handler threads; may take StateHub mutex via
 * GetSnapshot.
 *
 * @note No AUTONOMY_SMART_PTR_DEFINITIONS — this header declares no classes.
 * @see StateHub
 * @see rpc_system_handlers.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/context.hpp"
#include "autonomy/bridge/proto/bridge_options.pb.h"
#include <automsgs/rpcs/system.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Map configured identity into SystemService RobotIdentity.
 *
 * Copies fleet / inventory fields from RobotIdentityOptions into the RPC
 * message used by RobotFullInfo and related System handlers.
 *
 * @param[in] options Configured identity from bridge options / Context.
 * @param[out] identity Destination RobotIdentity (must be non-null).
 *
 * @warning @p identity must not be null; behavior on null is undefined.
 */
void FillRobotIdentity(const proto::RobotIdentityOptions& options,
                       ::automsgs::rpcs::system::RobotIdentity* identity);

/**
 * @brief Single source for SystemService Capabilities.
 *
 * Applies CapabilitiesOptions overrides onto the advertise bitmask / feature
 * flags exposed to cloud / app clients. Keep all capability mapping here so
 * handlers do not fork divergent defaults.
 *
 * @param[in] options Capability advertise overrides (may be default).
 * @param[out] caps Destination Capabilities message (must be non-null).
 *
 * @note Missing overrides leave implementation defaults from the .cc.
 * @warning @p caps must not be null; behavior on null is undefined.
 */
void FillRpcCapabilities(const proto::CapabilitiesOptions& options,
                         ::automsgs::rpcs::system::Capabilities* caps);

/**
 * @brief Aggregate the profile for SystemService RobotFullInfo.
 *
 * Combines Context::identity / capabilities with live StateHub / muxer /
 * domain readiness fields into a single Unary snapshot for onboarding.
 *
 * @param[in,out] context Bridge execution context (reads hub / options).
 * @return Populated RobotFullInfo value (by value for Unary handlers).
 *
 * @note May read StateHub::GetSnapshot; safe on handler threads.
 */
::automsgs::rpcs::system::RobotFullInfo BuildRpcRobotFullInfo(
    Context& context);

/**
 * @brief Map configured identity into System GetInfoResponse fields.
 *
 * Lighter than RobotFullInfo — fills the subset of identity fields exposed by
 * GetInfo without aggregating live motion / task state.
 *
 * @param[in] options Configured identity from bridge options / Context.
 * @param[out] response Destination GetInfoResponse (must be non-null).
 *
 * @warning @p response must not be null; behavior on null is undefined.
 */
void FillGetInfoResponse(const proto::RobotIdentityOptions& options,
                         ::automsgs::rpcs::system::GetInfoResponse* response);

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
