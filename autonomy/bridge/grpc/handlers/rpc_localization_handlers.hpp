/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_localization_handlers.hpp
 * @brief LocalizationService RpcHandlers: GetPose, GetStatus, SetInitialPose.
 *
 * @details
 * Unary-only service surface. Forwards to Context::localization()
 * (LocalizationStub). Stub reads `/amcl_pose` for GetPose and writes
 * @c kLocalizationGoal / reads @c kLocalizationFeedback for SetInitialPose.
 * No BRIDGE_LIFECYCLE / stream handlers in this file.
 *
 * Generated types (SMART_PTR via macros):
 * - RpcLocalizationGetPoseHandler — BRIDGE_UNARY GetPose
 * - RpcLocalizationGetStatusHandler — BRIDGE_GET GetStatus
 * - RpcLocalizationSetInitialPoseHandler — BRIDGE_STATUS SetInitialPose
 *
 * @par Invariants
 * - GetPose / GetStatus are unary reads from LocalizationStub.
 * - SetInitialPose returns common.Status from the stub.
 * - Handlers never touch Autolink; LocalizationStub owns pose cache + channel.
 * - Threading: gRPC completion queue; stub may take mutex_ / cache locks.
 *
 * @see LocalizationStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/localization.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief LocalizationService/GetPose — AMCL pose unary. */
BRIDGE_UNARY(RpcLocalizationGetPoseHandler,
                 ::automsgs::rpcs::localization::GetPoseRequest,
                 ::automsgs::rpcs::localization::GetPoseResponse,
                 "/automsgs.rpcs.localization.LocalizationService/GetPose",
                 &Context::localization, &clients::LocalizationStub::GetPose);

/** @brief LocalizationService/GetStatus — mirrored LocalizationStatus. */
BRIDGE_GET(
    RpcLocalizationGetStatusHandler, ::automsgs::rpcs::localization::GetStatusRequest,
    ::automsgs::rpcs::localization::LocalizationStatus,
    "/automsgs.rpcs.localization.LocalizationService/GetStatus",
    &Context::localization, &clients::LocalizationStub::GetStatus);

/** @brief LocalizationService/SetInitialPose — publish goal, return Status. */
BRIDGE_STATUS(
    RpcLocalizationSetInitialPoseHandler,
    ::automsgs::rpcs::localization::SetInitialPoseRequest,
    "/automsgs.rpcs.localization.LocalizationService/SetInitialPose",
    &Context::localization, &clients::LocalizationStub::SetInitialPose);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
