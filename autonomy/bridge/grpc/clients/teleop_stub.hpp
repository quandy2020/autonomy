/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file teleop_stub.hpp
 * @brief TeleopStub: Velocity (GoalChannel) + Drive/BackUp/Spin (Action).
 *
 * @details
 * Velocity uses GoalChannelCommandStub with TeleopGoalChannelTraits
 * (ShouldEmit / RejectReason). Relative motions (DriveOnHeading / BackUp /
 * Spin) delegate to TeleopRelativeBackend on the work scheduler. CancelGoal
 * cancels relative first, then publishes TELEOP_CMD_STOP on the GoalChannel.
 *
 * Topics (see bridge/constants.hpp):
 * - Goal: @c kTeleopGoal
 * - Feedback: @c kTeleopFeedback
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle; owns TeleopRelativeBackend UniquePtr.
 * CancelRegistry captures non-owning TeleopStub*. Muxer / scheduler /
 * idempotency are injected non-owning.
 *
 * @par Threading
 * Velocity Handle* on gRPC event thread (non-blocking GoalChannel).
 * Relative Handle* schedule Action work on WorkScheduler. GetSnapshot /
 * CancelGoal may touch both paths.
 *
 * @par Invariants
 * - CancelGoal cancels relative first, then GoalChannel stop.
 * - GetSnapshot prefers relative backend when busy.
 * - Pause/Resume/Cancel constants are all TELEOP_CMD_STOP on the wire.
 *
 * @see GoalChannelCommandStub
 * @see teleop::TeleopRelativeBackend
 * @see rpc_teleop_handlers.hpp
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autonomy/bridge/grpc/clients/goal_channel_command_stub.hpp"
#include "autonomy/bridge/grpc/clients/teleop_relative.hpp"
#include "autonomy/bridge/grpc/idempotency.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/bridge/constants.hpp"
#include <automsgs/rpcs/teleop.pb.h>
#include <automsgs/task/teleop.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief GoalChannel traits for TeleopTask (generated + TELEOP_CMD_STOP).
 *
 * @details
 * Declares Convert* / MakeResponse / IsTerminal (implemented in
 * teleop_stub.cpp). Pause/Resume/Cancel constants are all TELEOP_CMD_STOP.
 * Topics: @c kTeleopGoal / @c kTeleopFeedback. Muxer type TASK_TYPE_TELEOP.
 *
 * @note Request is VelocityRequest; Response is TeleopResponse.
 * @see TeleopGoalChannelTraits
 */
BRIDGE_CHANNEL_TRAITS(
    TeleopTraits,
    ::autonomy::task::proto::TeleopGoal,
    ::autonomy::task::proto::TeleopFeedback,
    ::automsgs::rpcs::teleop::VelocityRequest,
    ::automsgs::rpcs::teleop::TeleopResponse,
    ::autonomy::bridge::grpc::TASK_TYPE_TELEOP,
    ::autonomy::bridge::kTeleopGoal,
    ::autonomy::bridge::kTeleopFeedback,
    ::autonomy::task::proto::TELEOP_CMD_STOP,
    ::autonomy::task::proto::TELEOP_CMD_STOP,
    ::autonomy::task::proto::TELEOP_CMD_STOP);

/**
 * @brief TeleopTraits + Velocity emit / reject hooks.
 *
 * @details
 * ShouldEmit / RejectReason are implemented in teleop_stub.cpp and picked up
 * by GoalChannelStub via HasShouldEmit / HasRejectReason. Used as the Traits
 * parameter of GoalChannelCommandStub for TeleopStub.
 *
 * @see GoalChannelStub
 */
struct TeleopGoalChannelTraits : TeleopTraits {
    /**
     * @brief Shared / weak / unique pointer aliases for TeleopGoalChannelTraits.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopGoalChannelTraits)

    /**
     * @brief Gate Velocity feedback emission.
     *
     * @param[in] feedback       Teleop feedback frame.
     * @param[in] session_active Whether the GoalChannel session is active.
     * @return                   true when the frame should be converted and emitted.
     */
    static bool ShouldEmit(const Feedback& feedback, bool session_active);

    /**
     * @brief Optional pre-write reject for Velocity requests.
     *
     * @param[in] request        Velocity request.
     * @param[in] session_active Whether a session is already active.
     * @return                   Reject message, or nullopt to continue.
     */
    static std::optional<std::string> RejectReason(const Request& request,
                                                   bool session_active);
};

/**
 * @brief TeleopService: Velocity via GoalChannel; relative via Action backend.
 *
 * @details
 * Thin facade over GoalChannelCommandStub for Velocity, plus unique-owned
 * TeleopRelativeBackend for DriveOnHeading / BackUp / Spin. Handlers obtain
 * this stub via Context::teleop().
 *
 * @par Ownership
 * Owns relative_; DomainBundle owns this stub UniquePtr.
 *
 * @par Threading
 * Velocity path is non-blocking on the gRPC thread; relative path schedules
 * Action accept waits on WorkScheduler.
 *
 * @note relative_ is unique-owned; CancelRegistry captures non-owning TeleopStub*.
 * @see rpc_teleop_handlers.hpp
 */
class TeleopStub : public GoalChannelCommandStub<TeleopGoalChannelTraits>
{
public:
    /**
     * @brief Stream sink alias shared with TeleopRelativeBackend.
     */
    using StreamCallback = teleop::TeleopRelativeBackend::StreamCallback;

    /**
     * @brief Shared / weak / unique pointer aliases for TeleopStub.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopStub)

    /**
     * @brief Construct GoalChannel + TeleopRelativeBackend.
     *
     * @param[in] node        Autolink node.
     * @param[in] muxer       Shared task muxer.
     * @param[in] scheduler   Work pool for relative actions.
     * @param[in] idempotency Optional command-id cache.
     */
    TeleopStub(std::shared_ptr<autolink::Node> node,
               TaskMuxer::SharedPtr muxer, WorkScheduler* scheduler,
               CommandIdempotencyCache* idempotency = nullptr);

    /**
     * @brief Handle Velocity GoalChannel start / stream.
     *
     * @param[in] request         VelocityRequest.
     * @param[in] stream_callback Stream sink.
     * @return                    false if rejected before write.
     */
    bool HandleVelocity(const ::automsgs::rpcs::teleop::VelocityRequest& request,
                        StreamCallback stream_callback);

    /**
     * @brief Forward DriveOnHeading to the relative backend.
     *
     * @param[in] request  DriveOnHeadingRequest.
     * @param[in] callback Stream sink for accept / feedback / result.
     * @return             false if rejected before schedule.
     */
    bool HandleDriveOnHeading(
        const ::automsgs::rpcs::teleop::DriveOnHeadingRequest& request,
        StreamCallback callback) {
        return relative_->HandleDriveOnHeading(request, std::move(callback));
    }

    /**
     * @brief Forward BackUp to the relative backend.
     *
     * @param[in] request  BackUpRequest.
     * @param[in] callback Stream sink for accept / feedback / result.
     * @return             false if rejected before schedule.
     */
    bool HandleBackUp(const ::automsgs::rpcs::teleop::BackUpRequest& request,
                      StreamCallback callback) {
        return relative_->HandleBackUp(request, std::move(callback));
    }

    /**
     * @brief Forward Spin to the relative backend.
     *
     * @param[in] request  SpinRequest.
     * @param[in] callback Stream sink for accept / feedback / result.
     * @return             false if rejected before schedule.
     */
    bool HandleSpin(const ::automsgs::rpcs::teleop::SpinRequest& request,
                    StreamCallback callback) {
        return relative_->HandleSpin(request, std::move(callback));
    }

    /**
     * @brief Cancel relative and/or Velocity session.
     *
     * @details Cancels the relative backend first, then publishes stop and
     * clears the GoalChannel session.
     *
     * @param[in] goal_id Optional goal id filter.
     * @return            true if cancel succeeded / no-op.
     */
    bool CancelGoal(const std::string& goal_id = {});

    /**
     * @brief Pause relative motion (Velocity has no pause).
     *
     * @param[in] goal_id Optional goal id filter.
     * @return            true if pause was accepted / no-op.
     */
    bool PauseGoal(const std::string& goal_id = {}) {
        return relative_->PauseGoal(goal_id);
    }

    /**
     * @brief Resume relative motion.
     *
     * @param[in] goal_id Optional goal id filter.
     * @return            true if resume was accepted / no-op.
     */
    bool ResumeGoal(const std::string& goal_id = {}) {
        return relative_->ResumeGoal(goal_id);
    }

    /**
     * @brief Combined Velocity / relative status snapshot.
     *
     * @return TeleopResponse reflecting the active path.
     *
     * @note Prefers relative backend snapshot when IsBusy().
     */
    ::automsgs::rpcs::teleop::TeleopResponse GetSnapshot() const;

    /**
     * @brief Tear down any active teleop session (CancelGoal).
     */
    void ResetSession() { CancelGoal(); }

private:
    /**
     * @brief Owned Drive/BackUp/Spin Action backend.
     *
     * @details Constructed in the TeleopStub ctor; Velocity busy-check is
     * wired to the GoalChannel IsActive() path.
     */
    teleop::TeleopRelativeBackend::UniquePtr relative_{nullptr};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
