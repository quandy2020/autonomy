/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file exploration_stub.hpp
 * @brief ExplorationStub: GoalChannel adapter for ExplorationService → ExplorationTask.
 *
 * @details
 * Maps ExplorationService RPCs onto `/autonomy/task/exploration/{goal,feedback}`
 * (see @c kExplorationGoal / @c kExplorationFeedback in bridge/constants.hpp).
 * Legacy waypoint / finished channels (@c kExplorationWaypointChannel,
 * @c kExplorationFinishedChannel) are not owned by this stub.
 * Wire messages are `automsgs/task/exploration.pb.h` (`ExplorationGoal` /
 * `ExplorationFeedback`); Bridge does **not** include `autonomy/task` headers.
 *
 * @par Lifecycle
 * - Explore → HandleExplore → GoalChannelCommandStub::HandleRequest (also
 * caches last ExploreResponse under snapshot_mutex_)
 * - SetArea / SaveMap → side-channel WriteGoal (EXPLORATION_CMD_SET_AREA /
 * SAVE_MAP) without opening a stream
 * - Pause / Resume / Cancel → EXPLORATION_CMD_* via base WriteCommand
 *
 * @par Invariants
 * - Muxer slot type is TASK_TYPE_EXPLORATION.
 * - GetSnapshot returns a copy of last_response_ (never a reference).
 * - Convert* / HandleExplore / SetArea / SaveMap live in exploration_stub.cpp.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * HandleExplore / SetArea / SaveMap on gRPC threads; feedback
 * path updates last_response_ under snapshot_mutex_; GetSnapshot takes the
 * same mutex.
 *
 * @see GoalChannelCommandStub
 * @see rpc_explore_handlers.hpp
 */

#pragma once

#include <mutex>

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/exploration.pb.h>
#include <automsgs/task/exploration.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for ExplorationTask (generated + EXPLORATION_CMD_*).
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse /
 * IsTerminal (implemented in exploration_stub.cpp) and binds Pause/Resume/Cancel
 * to EXPLORATION_CMD_PAUSE / RESUME / CANCEL. Topics:
 * - Goal: @c kExplorationGoal (`/autonomy/task/exploration/goal`)
 * - Feedback: @c kExplorationFeedback (`/autonomy/task/exploration/feedback`)
 *
 * @note Request is ExploreRequest; Response is ExploreResponse.
 * @warning SetArea / SaveMap bypass HandleRequest and do not claim the muxer
 * via the streaming path; they WriteGoal directly.
 */
BRIDGE_CHANNEL_TRAITS(
    ExplorationTraits,
    ::autonomy::task::proto::ExplorationGoal,
    ::autonomy::task::proto::ExplorationFeedback,
    ::automsgs::rpcs::exploration::ExploreRequest,
    ::automsgs::rpcs::exploration::ExploreResponse,
    ::autonomy::bridge::grpc::TASK_TYPE_EXPLORATION,
    ::autonomy::bridge::kExplorationGoal,
    ::autonomy::bridge::kExplorationFeedback,
    ::autonomy::task::proto::EXPLORATION_CMD_PAUSE,
    ::autonomy::task::proto::EXPLORATION_CMD_RESUME,
    ::autonomy::task::proto::EXPLORATION_CMD_CANCEL);

/**
 * @brief ExplorationService Explore / SetArea / SaveMap / GetStatus via GoalChannel.
 *
 * @details
 * Facade over @ref GoalChannelCommandStub<ExplorationTraits> with an
 * extra snapshot cache for GetStatus (GetSnapshot). Streaming Explore updates
 * last_response_ on every relayed frame; SetArea / SaveMap publish one-shot
 * goals without a stream callback.
 *
 * @par Threading
 * snapshot_mutex_ protects last_response_; base GoalChannel state
 * has its own synchronization. Do not hold snapshot_mutex_ across WriteGoal.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @note Convert* / HandleExplore / SetArea / SaveMap live in
 * exploration_stub.cpp.
 *
 * @warning HandleExplore returns false if stream_callback is empty or gated.
 * @see rpc_explore_handlers.hpp
 */
class ExplorationStub : public GoalChannelCommandStub<ExplorationTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationStub)

    /**
     * @brief Inherit GoalChannelCommandStub constructors (node + muxer).
     */
    using GoalChannelCommandStub::GoalChannelCommandStub;

    /**
     * @brief Accept Explore and stream ExplorationTask feedback.
     *
     * Wraps the base HandleRequest sink so each relayed ExploreResponse is
     * also stored in last_response_ for GetSnapshot.
     *
     * @param[in] request         ExploreRequest (area, map_name, options, goal_id).
     * @param[in] stream_callback Sink for ACK / feedback / terminal frames.
     * @return                    false if @p stream_callback is empty or rejected before write.
     */
    bool HandleExplore(
        const ::automsgs::rpcs::exploration::ExploreRequest& request,
        StreamCallback stream_callback);

    /**
     * @brief Publish a set-area goal (non-streaming helper).
     *
     * Builds ExplorationGoal with EXPLORATION_CMD_SET_AREA and WriteGoal on
     * @c kExplorationGoal. Does not open a gRPC stream or update the snapshot
     * cache by itself.
     *
     * @param[in] request SetAreaRequest (area polygon / goal_id).
     *
     * @note AckHandler returns OK regardless of WriteGoal success.
     */
    void SetArea(const ::automsgs::rpcs::exploration::SetAreaRequest& request);

    /**
     * @brief Publish a save-map goal (non-streaming helper).
     *
     * Builds ExplorationGoal with EXPLORATION_CMD_SAVE_MAP and WriteGoal.
     *
     * @param[in] request SaveMapRequest (map_name / goal_id).
     *
     * @note AckHandler returns OK regardless of WriteGoal success.
     */
    void SaveMap(const ::automsgs::rpcs::exploration::SaveMapRequest& request);

    /**
     * @brief Copy of the last streamed ExploreResponse.
     *
     * @return Snapshot copy under snapshot_mutex_ (default-constructed if never
     *        updated).
     *
     * @note Safe to call from GetStatus handlers on gRPC threads.
     */
    ::automsgs::rpcs::exploration::ExploreResponse GetSnapshot() const;

private:
    /**
     * @brief Protects last_response_ for GetSnapshot / stream updates.
     *
     * @warning Do not hold across WriteGoal / HandleRequest.
     */
    mutable std::mutex snapshot_mutex_;

    /**
     * @brief Last ExploreResponse relayed on the Explore stream.
     *
     * @details Updated by HandleExplore's wrapped sink; GetSnapshot returns
     * a copy. Default-constructed until the first streamed frame.
     */
    ::automsgs::rpcs::exploration::ExploreResponse last_response_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
