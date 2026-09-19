/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file follow_stub.hpp
 * @brief FollowStub: GoalChannel adapter for TrackerTask (human follow).
 *
 * @details
 * Maps FollowService onto `/autonomy/task/tracking/{goal,feedback}`
 * (`kTrackingGoal` / `kTrackingFeedback`). Wire types:
 * `automsgs/task/tracker.pb.h` (`TrackerGoal` / `TrackerFeedback`).
 * Bridge does **not** include `autonomy/task` headers.
 *
 * @par Lifecycle
 * TRACKER_CMD_PAUSE / RESUME / CANCEL.
 * Muxer slot: TASK_TYPE_FOLLOW.
 * Convert* / MakeResponse / IsTerminal: follow_stub.cpp.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * gRPC event thread; no Action wait.
 *
 * @see GoalChannelCommandStub
 * @see rpc_follow_handlers.hpp
 */

#pragma once

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/follow.pb.h>
#include <automsgs/task/tracker.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for TrackerTask.
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse / IsTerminal
 * (implemented in follow_stub.cpp) and binds Pause/Resume/Cancel to
 * TRACKER_CMD_*. Topics: @c kTrackingGoal / @c kTrackingFeedback.
 *
 * @note Request is FollowRequest; Response is FollowResponse.
 * @warning Muxer type TASK_TYPE_FOLLOW must stay aligned with TaskServer.
 */
BRIDGE_CHANNEL_TRAITS(
    FollowTraits,
    ::autonomy::task::proto::TrackerGoal,
    ::autonomy::task::proto::TrackerFeedback,
    ::automsgs::rpcs::follow::FollowRequest,
    ::automsgs::rpcs::follow::FollowResponse,
    ::autonomy::bridge::grpc::TASK_TYPE_FOLLOW,
    ::autonomy::bridge::kTrackingGoal,
    ::autonomy::bridge::kTrackingFeedback,
    ::autonomy::task::proto::TRACKER_CMD_PAUSE,
    ::autonomy::task::proto::TRACKER_CMD_RESUME,
    ::autonomy::task::proto::TRACKER_CMD_CANCEL);

/**
 * @brief Forwards FollowService to TrackerTask via GoalChannel.
 *
 * @details
 * Thin facade: HandleFollow → HandleRequest. Requires a ready goal
 * writer; returns false when gated (estop / busy / reject).
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * gRPC event thread; no Action wait.
 */
class FollowStub : public GoalChannelCommandStub<FollowTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(FollowStub)

    /**
     * @brief Inherit GoalChannelCommandStub constructors (node + muxer).
     */
    using GoalChannelCommandStub::GoalChannelCommandStub;

    /**
     * @brief Accept a Follow start and stream Tracker feedback.
     *
     * @param[in] request         FollowRequest (goal_id, target, …).
     * @param[in] stream_callback Stream sink for ACK / feedback / terminal.
     * @return                    false if rejected before write (reject already emitted).
     */
    bool HandleFollow(const ::automsgs::rpcs::follow::FollowRequest& request,
                      StreamCallback stream_callback) {
        return HandleRequest(request, std::move(stream_callback));
    }
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
