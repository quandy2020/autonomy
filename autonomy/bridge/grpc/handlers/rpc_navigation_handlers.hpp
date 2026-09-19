/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rpc_navigation_handlers.hpp
 * @brief NavigationService RpcHandlers: Navigate stream, lifecycle, GetStatus.
 *
 * @details
 * Forwards to Context::navigator() (NavigatorStub). Stub GoalChannel topics
 * are @c kNavigationGoal / @c kNavigationFeedback
 * (`/autonomy/task/navigation/{goal,feedback}`). Navigate is BRIDGE_DECL with
 * an inline empty-waypoints reject before RelayStream; lifecycle uses
 * BRIDGE_LIFECYCLE; GetStatus uses NavigationGetStatusBuilder.
 *
 * Types:
 * - RpcNavigateHandler — BRIDGE_DECL + inline OnRequest (SMART_PTR via macro)
 * - RpcNavigationCancelHandler / RpcNavigationPauseHandler /
 *   RpcNavigationResumeHandler — BRIDGE_LIFECYCLE
 * - NavigationGetStatusBuilder — explicit struct with AUTONOMY_SMART_PTR_DEFINITIONS
 * - RpcNavigationGetStatusHandler — BRIDGE_BUILD NavigationGetStatusBuilder
 *
 * Invariants:
 * - Navigate rejects empty waypoints before RelayStream.
 * - Lifecycle uses NavigatorStub Cancel/Pause/Resume via BRIDGE_LIFECYCLE.
 * - GetStatus uses NavigationGetStatusBuilder (IsNavigating → RUNNING / IDLE).
 * - Ownership: per-RPC handlers; NavigatorStub owned by Context.
 * - Threading: gRPC completion queue; RelayStream sink may run off-thread.
 *
 * @see NavigatorStub
 * @see handler_templates.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/handlers/handler_templates.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/navigation.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/**
 * @brief NavigationService/Navigate — declare handler; OnRequest defined below.
 *
 * @note Rejects empty waypoints with INVALID_ARGUMENT / FAILED before calling
 *       NavigatorStub::HandleNavigate.
 */
BRIDGE_DECL(
    RpcNavigateHandler, ::automsgs::rpcs::navigation::NavigateRequest,
    autonomy::common::async_grpc::Stream<
        ::automsgs::rpcs::navigation::NavigateResponse>,
    "/automsgs.rpcs.navigation.NavigationService/Navigate");

/**
 * @brief NavigationService Cancel / Pause / Resume on GoalRequest.goal_id.
 */
BRIDGE_LIFECYCLE(RpcNavigation, ::automsgs::rpcs::navigation::GoalRequest,
                          "/automsgs.rpcs.navigation.NavigationService",
                          &Context::navigator, clients::NavigatorStub);

/**
 * @brief Builder for NavigationService/GetStatus unary response.
 *
 * @details Sets status=OK and state to NAVIGATION_STATE_RUNNING when the
 * navigator is active (IsNavigating), otherwise NAVIGATION_STATE_IDLE.
 * Used exclusively by RpcNavigationGetStatusHandler via BRIDGE_BUILD.
 *
 * @note Does not read Autolink; only Context::navigator() session flags.
 * @warning Requires non-null Context (BuildHandler / Reply path guarantees).
 */
struct NavigationGetStatusBuilder {
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigationGetStatusBuilder)

    /**
     * @brief Build a NavigateResponse status snapshot from Context.
     *
     * @param[in] context Bridge execution context (navigator stub).
     * @return            NavigateResponse with OK status and RUNNING or IDLE state.
     */
    static ::automsgs::rpcs::navigation::NavigateResponse Build(
        Context* context) {
        ::automsgs::rpcs::navigation::NavigateResponse response;
        *response.mutable_status() = OkStatus();
        response.set_state(
            context->navigator().IsNavigating()
                ? ::automsgs::rpcs::navigation::NAVIGATION_STATE_RUNNING
                : ::automsgs::rpcs::navigation::NAVIGATION_STATE_IDLE);
        return response;
    }
};

/** @brief NavigationService/GetStatus — NavigationGetStatusBuilder unary. */
BRIDGE_BUILD(
    RpcNavigationGetStatusHandler, ::automsgs::rpcs::navigation::GetStatusRequest,
    ::automsgs::rpcs::navigation::NavigateResponse,
    "/automsgs.rpcs.navigation.NavigationService/GetStatus",
    NavigationGetStatusBuilder);

/**
 * @brief Navigate OnRequest: validate waypoints then RelayStream to stub.
 *
 * @param[in] request NavigateRequest; waypoints_size() must be > 0.
 *
 * @note On empty waypoints replies a single NavigateResponse and returns
 *      without opening a streaming session.
 */
inline void RpcNavigateHandler::OnRequest(
    const ::automsgs::rpcs::navigation::NavigateRequest& request) {
    if (request.waypoints_size() == 0) {
        ::automsgs::rpcs::navigation::NavigateResponse response;
        *response.mutable_status() = ErrorStatus(
            ::automsgs::msgs::status_msgs::INVALID_ARGUMENT, "empty waypoints");
        response.set_state(
            ::automsgs::rpcs::navigation::NAVIGATION_STATE_FAILED);
        ReplyUnary(this, std::move(response));
        return;
    }
    LogIngress(RpcNavigateSignature::MethodName(), request);
    RelayStream(
        this,
        [&](auto* context, auto&& callback) {
            return context->navigator().HandleNavigate(request,
                                                       std::move(callback));
        },
        IsNavigateTerminal);
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
