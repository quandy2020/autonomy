/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_explore_handlers.hpp
 * @brief ExplorationService RpcHandlers: Explore stream, lifecycle, status,
 *        SetArea / SaveMap acks.
 *
 * @details
 * Forwards to Context::exploration() (ExplorationStub). Stub GoalChannel topics
 * are @c kExplorationGoal / @c kExplorationFeedback
 * (`/autonomy/task/exploration/{goal,feedback}`). SetArea / SaveMap are
 * side-channel WriteGoal helpers wrapped as AckHandler (void → OK Status).
 *
 * Generated types (SMART_PTR via macros):
 * - RpcExploreHandler — BRIDGE_STREAM HandleExplore / IsExploreTerminal
 * - RpcExploreCancelHandler / RpcExplorePauseHandler / RpcExploreResumeHandler — BRIDGE_LIFECYCLE
 * - RpcExploreGetStatusHandler — BRIDGE_GET GetSnapshot
 * - RpcExploreSetAreaHandler / RpcExploreSaveMapHandler — BRIDGE_ACK
 *
 * @par Invariants
 * - Explore finishes on IsExploreTerminal.
 * - SetArea / SaveMap are AckHandler (void stub → OK Status with fixed message).
 * - Handlers never own catalog / map state; ExplorationStub owns snapshot.
 * - Threading: gRPC completion queue; GetSnapshot takes stub snapshot_mutex_.
 *
 * @see ExplorationStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/exploration.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief OkStatus message for SetArea AckHandler. */
inline constexpr char kAreaUpdated[] = "area updated";
/** @brief OkStatus message for SaveMap AckHandler. */
inline constexpr char kSaveRequested[] = "save requested";

/** @brief ExplorationService/Explore — stream ExploreResponse until terminal. */
BRIDGE_STREAM(
    RpcExploreHandler, ::automsgs::rpcs::exploration::ExploreRequest,
    autonomy::common::async_grpc::Stream<
        ::automsgs::rpcs::exploration::ExploreResponse>,
    "/automsgs.rpcs.exploration.ExplorationService/Explore",
    &Context::exploration, &clients::ExplorationStub::HandleExplore,
    &IsExploreTerminal);

/**
 * @brief ExplorationService Cancel / Pause / Resume on GoalRequest.goal_id.
 */
BRIDGE_LIFECYCLE(RpcExplore, ::automsgs::rpcs::exploration::GoalRequest,
                          "/automsgs.rpcs.exploration.ExplorationService",
                          &Context::exploration, clients::ExplorationStub);

/** @brief ExplorationService/GetStatus — last streamed ExploreResponse snapshot. */
BRIDGE_GET(
    RpcExploreGetStatusHandler, ::automsgs::rpcs::exploration::GetStatusRequest,
    ::automsgs::rpcs::exploration::ExploreResponse,
    "/automsgs.rpcs.exploration.ExplorationService/GetStatus",
    &Context::exploration, &clients::ExplorationStub::GetSnapshot);

/** @brief ExplorationService/SetArea — ack after ExplorationStub::SetArea. */
BRIDGE_ACK(
    RpcExploreSetAreaHandler, ::automsgs::rpcs::exploration::SetAreaRequest,
    "/automsgs.rpcs.exploration.ExplorationService/SetArea",
    &Context::exploration, &clients::ExplorationStub::SetArea,
    kAreaUpdated);

/** @brief ExplorationService/SaveMap — ack after ExplorationStub::SaveMap. */
BRIDGE_ACK(
    RpcExploreSaveMapHandler, ::automsgs::rpcs::exploration::SaveMapRequest,
    "/automsgs.rpcs.exploration.ExplorationService/SaveMap",
    &Context::exploration, &clients::ExplorationStub::SaveMap,
    kSaveRequested);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
