/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file mapping_stub.hpp
 * @brief MappingStub: GoalChannel adapter for MappingTask (map load / session).
 *
 * @details
 * Maps StartMapping onto `/autonomy/task/mapping/{goal,feedback}`
 * (see @c kMappingGoal / @c kMappingFeedback in bridge/constants.hpp;
 * legacy aliases @c kMappingGoalChannel / @c kMappingFeedbackChannel).
 * Wire messages are `automsgs/task/mapping.pb.h` (`MappingGoal` /
 * `MappingFeedback`); Bridge does **not** include `autonomy/task` headers.
 *
 * MapServiceStub owns catalog / session UX and calls into this stub for
 * GoalChannel writes. Occupancy grid live samples use @c kMapChannel
 * (`/map`) in MapServiceStub, not here.
 *
 * @par Lifecycle
 * - HandleStart → ConvertToGoal (MAP_CMD_LOAD) → HandleRequest
 * - Feedback hook caches current_map_name_ under map_name_mutex_
 * - Pause / Resume / Cancel all map to MAP_CMD_CANCEL (no distinct pause)
 * - WriteMappingGoal exposes raw goal publish for MapServiceStub helpers
 *
 * @par Invariants
 * - Muxer slot type is TASK_TYPE_MAP.
 * - Convert* / MakeResponse / IsTerminal live in mapping_stub.cpp.
 * - GetCurrentMapName returns a copy under map_name_mutex_.
 * - HandleStart installs a no-op stream callback when none is provided
 * (unary StartMapping via MapServiceStub).
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; MapServiceStub holds MappingStub*;
 * CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * feedback hook runs on Autolink reader path and takes
 * map_name_mutex_; HandleStart / GetCurrentMapName may run on gRPC threads.
 *
 * @see GoalChannelCommandStub
 * @see MapServiceStub
 * @see rpc_map_handlers.hpp
 */

#pragma once

#include <mutex>
#include <string>

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/mapping.pb.h>
#include <automsgs/task/mapping.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for MappingTask (generated + MAP_CMD_CANCEL).
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse /
 * IsTerminal (implemented in mapping_stub.cpp). Pause/Resume/Cancel constants
 * are all MAP_CMD_CANCEL. Topics:
 * - Goal: @c kMappingGoal (`/autonomy/task/mapping/goal`)
 * - Feedback: @c kMappingFeedback (`/autonomy/task/mapping/feedback`)
 *
 * ConvertToGoal uses MAP_CMD_LOAD with optional map_name from StartMappingRequest.
 *
 * @note Request is StartMappingRequest; Response is MappingStatus.
 * @warning PauseGoal / ResumeGoal still cancel the mapping session on the wire.
 */
BRIDGE_CHANNEL_TRAITS(
    MappingTraits,
    ::autonomy::task::proto::MappingGoal,
    ::autonomy::task::proto::MappingFeedback,
    ::automsgs::rpcs::mapping::StartMappingRequest,
    ::automsgs::rpcs::mapping::MappingStatus,
    ::autonomy::bridge::grpc::TASK_TYPE_MAP,
    ::autonomy::bridge::kMappingGoal,
    ::autonomy::bridge::kMappingFeedback,
    ::autonomy::task::proto::MAP_CMD_CANCEL,
    ::autonomy::task::proto::MAP_CMD_CANCEL,
    ::autonomy::task::proto::MAP_CMD_CANCEL);

/**
 * @brief MappingTask GoalChannel client with current-map-name cache.
 *
 * @details
 * Extends @ref GoalChannelCommandStub<MappingTraits> with a feedback
 * hook that records MappingFeedback::current_map_name. MapServiceStub uses
 * HandleStart (often without a stream) and WriteMappingGoal for load /
 * set-current flows.
 *
 * @par Threading
 * map_name_mutex_ protects current_map_name_; do not hold it across
 * WriteGoal / HandleRequest.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; MapServiceStub holds non-owning MappingStub*.
 *
 * @note Convert* / IsTerminal live in mapping_stub.cpp.
 * @see MapServiceStub
 */
class MappingStub : public GoalChannelCommandStub<MappingTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MappingStub)

    /**
     * @brief Construct GoalChannel stub and install the map-name feedback hook.
     *
     * @param[in] node  Autolink node (goal writer + feedback reader).
     * @param[in] muxer Shared task muxer (TASK_TYPE_MAP slot).
     */
    MappingStub(std::shared_ptr<autolink::Node> node,
                TaskMuxer::SharedPtr muxer);

    /**
     * @brief Start / load mapping and optionally stream MappingStatus frames.
     *
     * Seeds current_map_name_ from @p request.map_name() when non-empty.
     * When @p stream_callback is empty, a no-op sink is installed so unary
     * MapService callers can fire-and-forget HandleRequest.
     *
     * @param[in] request         StartMappingRequest (goal_id, map_name, …).
     * @param[in] stream_callback Optional sink; default no-op.
     * @return                    false if gated out before write (reject already emitted).
     */
    bool HandleStart(
        const ::automsgs::rpcs::mapping::StartMappingRequest& request,
        StreamCallback stream_callback = {});

    /**
     * @brief Latest map name observed from feedback or HandleStart seed.
     *
     * @return Copy of current_map_name_ under map_name_mutex_.
     */
    std::string GetCurrentMapName() const;

    /**
     * @brief Publish a raw MappingGoal on the GoalChannel.
     *
     * Escape hatch for MapServiceStub / tests that already built a goal.
     *
     * @param[in] goal MappingGoal to write.
     * @return         true if the goal writer accepted the publish.
     *
     * @warning Bypasses HandleRequest muxer claim / stream setup; callers must
     * understand session semantics.
     */
    bool WriteMappingGoal(const ::autonomy::task::proto::MappingGoal& goal) {
        return channel_.WriteGoal(goal);
    }

private:
    /**
     * @brief Protects current_map_name_ for feedback hook / getters.
     *
     * @warning Do not hold across WriteGoal / HandleRequest.
     */
    mutable std::mutex map_name_mutex_;

    /**
     * @brief Latest map name from MappingFeedback or HandleStart seed.
     *
     * @details Seeded from StartMappingRequest::map_name when non-empty;
     * updated by the feedback hook installed in the constructor.
     */
    std::string current_map_name_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
