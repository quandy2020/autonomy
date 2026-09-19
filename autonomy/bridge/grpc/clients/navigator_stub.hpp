/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file navigator_stub.hpp
 * @brief NavigatorStub: GoalChannel adapter for NavigationService → NavigationTask.
 *
 * @details
 * Maps NavigationService RPCs onto `/autonomy/task/navigation/{goal,feedback}`
 * (see @c kNavigationGoal / @c kNavigationFeedback in bridge/constants.hpp).
 * Wire messages are `automsgs/task/navigation.pb.h` (`NavigationGoal` /
 * `NavigationFeedback`); Bridge does **not** include `autonomy/task` headers.
 *
 * @par Lifecycle
 * - Navigate → HandleNavigate → GoalChannelCommandStub::HandleRequest
 * - Pause / Resume / Cancel → NAV_CMD_* via base WriteCommand
 * - ReplanGoal → NAV_CMD_REPLAN without clearing the session
 *
 * @par Invariants
 * - Muxer slot type is TASK_TYPE_NAVIGATION.
 * - Convert* / MakeResponse / IsTerminal live in navigator_stub.cpp.
 * - IsNavigating() is a thin alias of IsActive() for handler builders.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * intended for the gRPC event thread; feedback is relayed
 * asynchronously while IsActive(). Must not block on Action results.
 *
 * @see GoalChannelCommandStub
 * @see rpc_navigation_handlers.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/navigation.pb.h>
#include <automsgs/task/navigation.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for NavigationTask (generated + NAV_CMD_*).
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse /
 * IsTerminal (implemented in navigator_stub.cpp) and binds Pause/Resume/Cancel
 * to NAV_CMD_PAUSE / RESUME / CANCEL. Topics:
 * - Goal: @c kNavigationGoal (`/autonomy/task/navigation/goal`)
 * - Feedback: @c kNavigationFeedback (`/autonomy/task/navigation/feedback`)
 *
 * @note Request type is NavigateRequest; Response is NavigateResponse.
 * @warning Muxer type TASK_TYPE_NAVIGATION must stay aligned with TaskServer.
 */
BRIDGE_CHANNEL_TRAITS(
    NavigatorTraits,
    ::autonomy::task::proto::NavigationGoal,
    ::autonomy::task::proto::NavigationFeedback,
    ::automsgs::rpcs::navigation::NavigateRequest,
    ::automsgs::rpcs::navigation::NavigateResponse,
    ::autonomy::bridge::grpc::TASK_TYPE_NAVIGATION,
    ::autonomy::bridge::kNavigationGoal,
    ::autonomy::bridge::kNavigationFeedback,
    ::autonomy::task::proto::NAV_CMD_PAUSE,
    ::autonomy::task::proto::NAV_CMD_RESUME,
    ::autonomy::task::proto::NAV_CMD_CANCEL);

/**
 * @brief NavigationService Navigate / lifecycle via GoalChannel.
 *
 * @details
 * Thin facade over @ref GoalChannelCommandStub<NavigatorTraits>.
 * Handlers call HandleNavigate for streams and inherit PauseGoal / ResumeGoal /
 * CancelGoal from the base. ReplanGoal is navigation-specific (no session
 * clear).
 *
 * @par Threading
 * gRPC / control path; feedback callbacks may run on Autolink
 * reader threads and are marshalled into the stream sink by the base.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @note Convert* / MakeResponse / IsTerminal live in navigator_stub.cpp.
 * @warning HandleNavigate returns false if gated out before write; reject
 * frames (if any) are already emitted by the base.
 *
 * @see rpc_navigation_handlers.hpp
 */
class NavigatorStub : public GoalChannelCommandStub<NavigatorTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigatorStub)

    /**
     * @brief Inherit GoalChannelCommandStub constructors (node + muxer).
     */
    using GoalChannelCommandStub::GoalChannelCommandStub;

    /**
     * @brief Accept Navigate and stream NavigationTask feedback.
     *
     * Converts @p request to NavigationGoal, claims the muxer slot, writes the
     * goal on @c kNavigationGoal, and relays feedback until IsTerminal.
     *
     * @param[in] request         NavigateRequest (waypoints, goal_id, options, …).
     * @param[in] stream_callback Sink for ACK / feedback / terminal frames.
     * @return                    false if rejected before write (reject already emitted).
     *
     * @note Empty-waypoint validation is performed by RpcNavigateHandler, not
     * this stub.
     */
    bool HandleNavigate(
        const ::automsgs::rpcs::navigation::NavigateRequest& request,
        StreamCallback stream_callback) {
        return HandleRequest(request, std::move(stream_callback));
    }

    /**
     * @brief Whether a navigation session is currently active.
     *
     * @return true when the GoalChannel session IsActive().
     *
     * @note Used by NavigationGetStatusBuilder to map RUNNING vs IDLE.
     */
    bool IsNavigating() const { return IsActive(); }

    /**
     * @brief Publish NAV_CMD_REPLAN without clearing the session.
     *
     * Forces the NavigationTask to replan while keeping the active goal /
     * muxer claim. Does not finish the gRPC stream.
     *
     * @param[in] goal_id Unused; reserved for API symmetry with Cancel/Pause.
     *
     * @warning No-op if the goal writer is not ready; does not report failure.
     */
    void ReplanGoal(const std::string& /*goal_id*/ = {}) {
        WriteCommand(::autonomy::task::proto::NAV_CMD_REPLAN);
    }
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
