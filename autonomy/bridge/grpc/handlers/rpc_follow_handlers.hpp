/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_follow_handlers.hpp
 * @brief FollowService RpcHandlers: Follow stream, lifecycle, GetStatus.
 *
 * @details
 * Forwards to Context::follow() (FollowStub → TrackerTask GoalChannel).
 * Stub topics are @c kTrackingGoal / @c kTrackingFeedback
 * (`/autonomy/task/tracking/{goal,feedback}`). Handlers do not include
 * autonomy/task headers.
 *
 * Generated types (SMART_PTR via macros):
 * - RpcFollowHandler — BRIDGE_STREAM HandleFollow / IsFollowTerminal
 * - RpcFollowCancelHandler / RpcFollowPauseHandler / RpcFollowResumeHandler — BRIDGE_LIFECYCLE
 * - RpcFollowGetStatusHandler — BRIDGE_BUILD ActiveStatus (FOLLOWING vs IDLE)
 *
 * @par Invariants
 * - Follow finishes on IsFollowTerminal.
 * - GetStatus uses ActiveStatus (FOLLOWING vs IDLE).
 * - Ownership: per-RPC handler instances; FollowStub owned by Context.
 * - Threading: gRPC completion queue; feedback relayed async by the stub.
 *
 * @see FollowStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/rpcs/follow.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/** @brief FollowService/Follow — stream FollowResponse until terminal. */
BRIDGE_STREAM(
    RpcFollowHandler, ::automsgs::rpcs::follow::FollowRequest,
    autonomy::common::async_grpc::Stream<::automsgs::rpcs::follow::FollowResponse>,
    "/automsgs.rpcs.follow.FollowService/Follow", &Context::follow,
    &clients::FollowStub::HandleFollow, &IsFollowTerminal);

/**
 * @brief FollowService Cancel / Pause / Resume on GoalRequest.goal_id.
 */
BRIDGE_LIFECYCLE(RpcFollow, ::automsgs::rpcs::follow::GoalRequest,
                          "/automsgs.rpcs.follow.FollowService",
                          &Context::follow, clients::FollowStub);

/**
 * @brief FollowService/GetStatus — ActiveStatus FOLLOWING when stub IsActive().
 */
BRIDGE_BUILD(
    RpcFollowGetStatusHandler, ::automsgs::rpcs::follow::GetStatusRequest,
    ::automsgs::rpcs::follow::FollowResponse,
    "/automsgs.rpcs.follow.FollowService/GetStatus",
    ActiveStatus<
        ::automsgs::rpcs::follow::FollowResponse, &Context::follow,
        ::automsgs::rpcs::follow::FOLLOW_STATE_FOLLOWING,
        ::automsgs::rpcs::follow::FOLLOW_STATE_IDLE>);

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
