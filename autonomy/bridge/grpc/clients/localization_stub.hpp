/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file localization_stub.hpp
 * @brief LocalizationStub: GoalChannel + AMCL pose cache for LocalizationService.
 *
 * @details
 * Combines two data paths:
 * 1. GoalChannel on `/autonomy/task/localization/{goal,feedback}`
 * (@c kLocalizationGoal / @c kLocalizationFeedback) for SetInitialPose
 * (LOCALIZATION_CMD_SET_INITIAL_POSE).
 * 2. LatestMessageCache on `/amcl_pose` for GetPose samples
 * (PoseWithCovarianceStamped).
 *
 * Wire messages are `automsgs/task/localization.pb.h`; Bridge does **not**
 * include `autonomy/task` headers. Muxer type is TASK_TYPE_NONE (localization
 * does not claim a robot task slot like navigation).
 *
 * @par Lifecycle
 * - SetInitialPose → WriteGoal (not a full HandleRequest stream)
 * - Feedback hook updates state_ / quality_ under mutex_
 * - AMCL reader marks UNKNOWN → LOCALIZED on first pose
 * - Pause / Resume / Cancel all map to LOCALIZATION_CMD_STOP
 *
 * @par Invariants
 * - GetPose / GetStatus return copies; never references into caches.
 * - SetInitialPose requires request.has_pose(); else INVALID_ARGUMENT.
 * - Convert* / IsTerminal / HandleFeedback live in localization_stub.cpp.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; CancelRegistry captures non-owning Stub*.
 *
 * @par Threading
 * AMCL + feedback callbacks take mutex_; GetPose / GetStatus /
 * SetInitialPose may run on gRPC threads and take the same mutex for state_
 * / quality_ only (pose_cache_ has its own synchronization).
 *
 * @see GoalChannelCommandStub
 * @see LatestMessageCache
 * @see rpc_localization_handlers.hpp
 */

#pragma once

#include <mutex>
#include <string>

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/grpc/clients/latest_message_cache.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include <automsgs/rpcs/localization.pb.h>
#include <automsgs/task/localization.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for LocalizationTask (generated + STOP).
 *
 * @details
 * Declares ConvertToGoal / ConvertFromFeedback / MakeResponse /
 * IsTerminal (implemented in localization_stub.cpp). Pause/Resume/Cancel
 * constants are all LOCALIZATION_CMD_STOP. Topics:
 * - Goal: @c kLocalizationGoal (`/autonomy/task/localization/goal`)
 * - Feedback: @c kLocalizationFeedback (`/autonomy/task/localization/feedback`)
 *
 * ConvertToGoal uses LOCALIZATION_CMD_SET_INITIAL_POSE from SetInitialPoseRequest.
 *
 * @note Muxer type TASK_TYPE_NONE — does not occupy navigation/teleop slots.
 * @warning PauseGoal / ResumeGoal still publish STOP on the wire.
 */
BRIDGE_CHANNEL_TRAITS(
    LocalizationTraits,
    ::autonomy::task::proto::LocalizationGoal,
    ::autonomy::task::proto::LocalizationFeedback,
    ::automsgs::rpcs::localization::SetInitialPoseRequest,
    ::automsgs::rpcs::localization::LocalizationStatus,
    ::autonomy::bridge::grpc::TASK_TYPE_NONE,
    ::autonomy::bridge::kLocalizationGoal,
    ::autonomy::bridge::kLocalizationFeedback,
    ::autonomy::task::proto::LOCALIZATION_CMD_STOP,
    ::autonomy::task::proto::LOCALIZATION_CMD_STOP,
    ::autonomy::task::proto::LOCALIZATION_CMD_STOP);

/**
 * @brief LocalizationService: GetPose / GetStatus cache + SetInitialPose GoalChannel.
 *
 * @details
 * Owns an AMCL pose LatestMessageCache and mirrors LocalizationFeedback
 * into LocalizationState / quality for GetStatus. SetInitialPose writes a goal
 * and returns common.Status (unary), rather than opening a Bridge stream.
 *
 * @par Threading
 * mutex_ protects state_ / quality_; pose_cache_ is internally
 * synchronized. Feedback hook and AMCL bind callbacks may run off the gRPC
 * thread.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle.
 *
 * @note Convert* / HandleFeedback live in localization_stub.cpp.
 * @see rpc_localization_handlers.hpp
 */
class LocalizationStub : public GoalChannelCommandStub<LocalizationTraits>
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(LocalizationStub)

    /**
     * @brief Construct GoalChannel + AMCL pose reader + feedback state hook.
     *
     * @param[in] node  Autolink node (required for pose cache bind).
     * @param[in] muxer Optional muxer (TASK_TYPE_NONE; may be null).
     */
    LocalizationStub(std::shared_ptr<autolink::Node> node,
                     TaskMuxer::SharedPtr muxer = nullptr);

    /**
     * @brief Return the latest AMCL pose (optional frame_id override).
     *
     * Reads pose_cache_; on miss returns LOCALIZATION_UNAVAILABLE. When
     * @p request.map_frame_id() is non-empty, overwrites the pose header
     * frame_id. Confidence comes from the last feedback quality_ when > 0.
     *
     * @param[in] request GetPoseRequest (optional map_frame_id).
     * @return            GetPoseResponse with status OK or LOCALIZATION_UNAVAILABLE.
     */
    ::automsgs::rpcs::localization::GetPoseResponse GetPose(
        const ::automsgs::rpcs::localization::GetPoseRequest& request) const;

    /**
     * @brief Snapshot of mirrored localization state / confidence.
     *
     * @return LocalizationStatus with state_ and optional confidence.
     *
     * @note Does not query AMCL; only the feedback / bootstrap mirror.
     */
    ::automsgs::rpcs::localization::LocalizationStatus GetStatus() const;

    /**
     * @brief Publish SetInitialPose goal on the localization GoalChannel.
     *
     * Validates pose presence, sets state_ to INITIALIZING, writes the goal.
     *
     * @param[in] request SetInitialPoseRequest (must include pose).
     * @return            OkStatus on publish success; INVALID_ARGUMENT / INTERNAL on failure.
     *
     * @warning Not a streaming RPC; callers do not receive feedback frames here.
     */
    ::automsgs::rpcs::common::Status SetInitialPose(
        const ::automsgs::rpcs::localization::SetInitialPoseRequest& request);

private:
    /**
     * @brief AMCL pose message type cached from `/amcl_pose`.
     */
    using PoseMsg = ::automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped;

    /**
     * @brief Mirror LocalizationFeedback into state_ / quality_ under mutex_.
     *
     * @param[in] feedback Latest LocalizationFeedback from GoalChannel.
     */
    void HandleFeedback(const LocalizationTraits::Feedback& feedback);

    /**
     * @brief Autolink node used to bind the AMCL pose reader.
     *
     * @details Shared ownership; required for pose_cache_.BindReader.
     */
    std::shared_ptr<autolink::Node> node_{nullptr};

    /**
     * @brief Latest `/amcl_pose` sample (internally synchronized).
     *
     * @details Used by GetPose; first message may promote state_ UNKNOWN →
     * LOCALIZED via the bind hook.
     */
    LatestMessageCache<PoseMsg> pose_cache_;

    /**
     * @brief Protects state_ / quality_ mirrors (not pose_cache_).
     */
    mutable std::mutex mutex_;

    /**
     * @brief Mirrored localization state for GetStatus / SetInitialPose.
     *
     * @details Updated from GoalChannel feedback and AMCL bootstrap hooks.
     */
    ::automsgs::rpcs::localization::LocalizationState state_{
        ::automsgs::rpcs::localization::LOCALIZATION_STATE_UNKNOWN};

    /**
     * @brief Last feedback quality / confidence (0 when unknown).
     *
     * @details Used by GetPose when > 0 to populate confidence fields.
     */
    float quality_{0.f};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
