/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file teleop_relative.hpp
 * @brief Drive / BackUp / Spin Action CRTP + TeleopRelativeBackend.
 *
 * @details
 * RelativeActionBase supplies shared accept/reject/result mapping; leaf
 * traits implement ConvertToGoal / MakeFeedback (bodies in teleop_relative.cpp).
 * TeleopRelativeBackend owns the three NodeClients, session, and muxer
 * interaction for TeleopStub.
 *
 * @par Ownership
 * TeleopRelativeBackend UniquePtr owned by TeleopStub. Action clients are
 * SharedPtr; goal handles are shared while a mode is active. Muxer /
 * scheduler / idempotency are injected non-owning.
 *
 * @par Threading
 * Public Handle* / Cancel* / GetSnapshot share mutex_; action callbacks
 * clear handles under lock then emit outside. Accept waits run on
 * WorkScheduler via ActionBackgroundInterface.
 *
 * @par Invariants
 * - At most one relative mode is active (Mode enum under mutex_).
 * - StartRelativeAction uses ActionBackgroundInterface on the work pool.
 * - Cancel/Pause/Resume operate on the active goal handle when present.
 *
 * @see TeleopStub
 * @see ActionBackgroundInterface
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "autolink/action/types.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/action_background_interface.hpp"
#include "autonomy/bridge/grpc/clients/action_send.hpp"
#include "autonomy/bridge/grpc/idempotency.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include "autonomy/bridge/grpc/session.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "autonomy/bridge/node_client.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include <automsgs/rpcs/teleop.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace teleop {
namespace detail {

/**
 * @brief Alias for status_msgs StatusCode used in MakeTeleopResponse.
 */
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

/**
 * @brief Shorthand for nav action protobuf namespace.
 */
namespace nav_actions = ::automsgs::actions;

/**
 * @brief Shorthand for teleop RPC protobuf namespace.
 */
namespace teleop_rpc = ::automsgs::rpcs::teleop;

/**
 * @brief Map action ResultCode to TeleopState for terminal frames.
 *
 * @param[in] code Action result code.
 * @return         SUCCEEDED / CANCELLED / FAILED.
 */
inline teleop_rpc::TeleopState ResolveResultState(
    autolink::action::ResultCode code) {
    if (code == autolink::action::ResultCode::SUCCEEDED) {
        return teleop_rpc::TELEOP_STATE_SUCCEEDED;
    }
    if (code == autolink::action::ResultCode::CANCELED) {
        return teleop_rpc::TELEOP_STATE_CANCELLED;
    }
    return teleop_rpc::TELEOP_STATE_FAILED;
}

/**
 * @brief Build a TeleopResponse with status overlay.
 *
 * @param[in] goal_id Goal id to stamp.
 * @param[in] state   TeleopState enum.
 * @param[in] ok      Success bit for status.
 * @param[in] detail  Optional detail text.
 * @return            Populated TeleopResponse.
 */
inline teleop_rpc::TeleopResponse MakeTeleopResponse(
    const std::string& goal_id, teleop_rpc::TeleopState state, bool ok,
    const std::string& detail = "") {
    teleop_rpc::TeleopResponse response;
    response.set_goal_id(goal_id);
    response.set_state(state);
    response.set_detail(detail);
    *response.mutable_status() =
        ok ? OkStatus(detail)
           : ErrorStatus(StatusCode::TELEOP_BUSY, detail);
    return response;
}

/**
 * @brief Build a REJECTED TeleopResponse for accept timeout / rejection.
 *
 * @param[in] goal_id Goal id to stamp.
 * @param[in] status  Accept outcome (timeout vs rejected).
 * @param[in] verb    Short verb from Traits::Verb() (drive / backup / spin).
 * @return            REJECTED TeleopResponse.
 */
inline teleop_rpc::TeleopResponse MakeAcceptFailureResponse(
    const std::string& goal_id, ActionAcceptStatus status, const char* verb) {
    const char* detail = status == ActionAcceptStatus::kTimeout
                             ? " goal accept timeout"
                             : " goal rejected";
    return MakeTeleopResponse(goal_id, teleop_rpc::TELEOP_STATE_REJECTED, false,
                              std::string(verb) + detail);
}

}  // namespace detail

/**
 * @brief Shared teleop relative-motion Action CRTP layer.
 *
 * @tparam Derived DriveOnHeading / BackUp / Spin leaf.
 *
 * @note kEmitAfterAccept is false; accept ACK is the only immediate frame.
 */
template <typename Derived>
struct RelativeActionBase : ActionBackgroundInterface<Derived> {
    AUTONOMY_SMART_PTR_DEFINITIONS(RelativeActionBase<Derived>)

    /**
     * @brief Teleop stream response type shared by all relative leaves.
     */
    using Response = ::automsgs::rpcs::teleop::TeleopResponse;

    static constexpr bool kEmitAfterAccept = false;
    static constexpr auto kAcceptTimeout = std::chrono::seconds(10);

    /**
     * @brief Extract command id from a relative request.
     *
     * @tparam Request Relative RPC request type with goal_id().
     */
    template <typename Request>
    std::string CmdId(const Request& request) const {
        return request.goal_id();
    }

    /**
     * @brief Client id (unused → empty).
     *
     * @tparam Request Relative RPC request type.
     */
    template <typename Request>
    std::string ClientId(const Request&) const {
        return {};
    }

    /**
     * @brief Immediate ACTIVE accept ACK.
     *
     * @tparam Request Relative RPC request type with goal_id().
     */
    template <typename Request>
    Response MakeAccept(const Request& request) const {
        return detail::MakeTeleopResponse(
            request.goal_id(), detail::teleop_rpc::TELEOP_STATE_ACTIVE, true,
            "accepted");
    }

    /**
     * @brief REJECTED frame with detail message.
     */
    template <typename Request>
    Response MakeReject(const Request& request,
                        const std::string& message) const {
        return detail::MakeTeleopResponse(
            request.goal_id(), detail::teleop_rpc::TELEOP_STATE_REJECTED, false,
            message);
    }

    /**
     * @brief Map wrapped action result to a terminal TeleopResponse.
     *
     * @tparam Request        Relative RPC request type with goal_id().
     * @tparam WrappedResult  Action GoalHandle::WrappedResult (deduced).
     */
    template <typename Request, typename WrappedResult>
    Response MakeResult(const Request& request,
                        const WrappedResult& wrapped) const {
        const bool ok =
            wrapped.code == autolink::action::ResultCode::SUCCEEDED;
        return detail::MakeTeleopResponse(
            request.goal_id(), detail::ResolveResultState(wrapped.code), ok);
    }

    /**
     * @brief Accept-failure reject using Derived::Verb().
     */
    template <typename Request>
    Response MakeAcceptFailure(const Request& request,
                               ActionAcceptStatus status) const {
        return detail::MakeAcceptFailureResponse(request.goal_id(), status,
                                                 this->impl().Verb());
    }
};

/**
 * @brief DriveOnHeading action policy (CRTP leaf).
 *
 * ConvertToGoal / MakeFeedback are defined in teleop_relative.cpp.
 */
struct DriveOnHeadingTraits : RelativeActionBase<DriveOnHeadingTraits> {
    AUTONOMY_SMART_PTR_DEFINITIONS(DriveOnHeadingTraits)

    /**
     * @brief NodeClient for DriveOnHeadingAction.
     */
    using Client =
        ::autonomy::bridge::NodeClient<detail::nav_actions::DriveOnHeadingAction>;

    /**
     * @brief DriveOnHeading RPC request type.
     */
    using Request = detail::teleop_rpc::DriveOnHeadingRequest;

    /**
     * @brief Teleop stream response type.
     */
    using Response = ::automsgs::rpcs::teleop::TeleopResponse;

    /**
     * @brief Action goal type from Client.
     */
    using Goal = typename Client::Goal;

    /**
     * @brief Action feedback type from Client.
     */
    using Feedback = typename Client::Feedback;

    /**
     * @brief Short verb used in accept-failure messages ("drive").
     *
     * @return C-string verb.
     */
    const char* Verb() const { return "drive"; }

    /**
     * @brief Reject detail when the drive_on_heading server is not ready.
     *
     * @return Human-readable not-ready message.
     */
    const char* ServerNotReadyMessage() const {
        return "drive_on_heading server not ready";
    }

    /**
     * @brief Convert DriveOnHeadingRequest to an action Goal.
     *
     * @param[in] request DriveOnHeading RPC request.
     * @return            Action goal (implemented in teleop_relative.cpp).
     */
    Goal ConvertToGoal(const Request& request) const;

    /**
     * @brief Map action feedback to a TeleopResponse frame.
     *
     * @param[in] request  Source request.
     * @param[in] feedback Action feedback.
     * @return             Stream feedback frame (implemented in .cpp).
     */
    Response MakeFeedback(const Request& request,
                          const Feedback& feedback) const;
};

/**
 * @brief BackUp action policy (CRTP leaf).
 */
struct BackUpTraits : RelativeActionBase<BackUpTraits> {
    AUTONOMY_SMART_PTR_DEFINITIONS(BackUpTraits)

    /**
     * @brief NodeClient for BackUpAction.
     */
    using Client =
        ::autonomy::bridge::NodeClient<detail::nav_actions::BackUpAction>;

    /**
     * @brief BackUp RPC request type.
     */
    using Request = detail::teleop_rpc::BackUpRequest;

    /**
     * @brief Teleop stream response type.
     */
    using Response = ::automsgs::rpcs::teleop::TeleopResponse;

    /**
     * @brief Action goal type from Client.
     */
    using Goal = typename Client::Goal;

    /**
     * @brief Action feedback type from Client.
     */
    using Feedback = typename Client::Feedback;

    /**
     * @brief Short verb used in accept-failure messages ("backup").
     *
     * @return C-string verb.
     */
    const char* Verb() const { return "backup"; }

    /**
     * @brief Reject detail when the backup server is not ready.
     *
     * @return Human-readable not-ready message.
     */
    const char* ServerNotReadyMessage() const {
        return "backup server not ready";
    }

    /**
     * @brief Convert BackUpRequest to an action Goal.
     *
     * @param[in] request BackUp RPC request.
     * @return            Action goal (implemented in teleop_relative.cpp).
     */
    Goal ConvertToGoal(const Request& request) const;

    /**
     * @brief Map action feedback to a TeleopResponse frame.
     *
     * @param[in] request  Source request.
     * @param[in] feedback Action feedback.
     * @return             Stream feedback frame (implemented in .cpp).
     */
    Response MakeFeedback(const Request& request,
                          const Feedback& feedback) const;
};

/**
 * @brief Spin action policy (CRTP leaf).
 */
struct SpinTraits : RelativeActionBase<SpinTraits> {
    AUTONOMY_SMART_PTR_DEFINITIONS(SpinTraits)

    /**
     * @brief NodeClient for SpinAction.
     */
    using Client =
        ::autonomy::bridge::NodeClient<detail::nav_actions::SpinAction>;

    /**
     * @brief Spin RPC request type.
     */
    using Request = detail::teleop_rpc::SpinRequest;

    /**
     * @brief Teleop stream response type.
     */
    using Response = ::automsgs::rpcs::teleop::TeleopResponse;

    /**
     * @brief Action goal type from Client.
     */
    using Goal = typename Client::Goal;

    /**
     * @brief Action feedback type from Client.
     */
    using Feedback = typename Client::Feedback;

    /**
     * @brief Short verb used in accept-failure messages ("spin").
     *
     * @return C-string verb.
     */
    const char* Verb() const { return "spin"; }

    /**
     * @brief Reject detail when the spin server is not ready.
     *
     * @return Human-readable not-ready message.
     */
    const char* ServerNotReadyMessage() const {
        return "spin server not ready";
    }

    /**
     * @brief Convert SpinRequest to an action Goal.
     *
     * @param[in] request Spin RPC request.
     * @return            Action goal (implemented in teleop_relative.cpp).
     */
    Goal ConvertToGoal(const Request& request) const;

    /**
     * @brief Map action feedback to a TeleopResponse frame.
     *
     * @param[in] request  Source request.
     * @param[in] feedback Action feedback.
     * @return             Stream feedback frame (implemented in .cpp).
     */
    Response MakeFeedback(const Request& request,
                          const Feedback& feedback) const;
};

/**
 * @brief Owns Drive/BackUp/Spin Action clients and session state.
 *
 * @details
 * TeleopStub forwards relative RPCs here. At most one of Drive / BackUp /
 * Spin is active (Mode). Velocity busy-check (SetVelocityBusyCheck) blocks
 * starting relative while GoalChannel Velocity is active.
 *
 * @par Ownership
 * UniquePtr owned by TeleopStub; owns SharedPtr action clients and session_.
 * Muxer / scheduler / idempotency are non-owning.
 *
 * @par Threading
 * Public Handle* / Cancel* / GetSnapshot share mutex_; action callbacks
 * clear handles under lock then emit outside.
 *
 * @warning SetVelocityBusyCheck must remain valid for the backend lifetime.
 * @see TeleopStub
 */
class TeleopRelativeBackend
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for TeleopRelativeBackend.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TeleopRelativeBackend)

    /**
     * @brief Stream sink for TeleopResponse accept / feedback / result frames.
     */
    using StreamCallback = std::function<void(
        const ::automsgs::rpcs::teleop::TeleopResponse& response)>;

    /**
     * @brief Predicate reporting whether Velocity GoalChannel is busy.
     */
    using BusyCheck = std::function<bool()>;

    /**
     * @brief Construct clients bound to @p node and shared muxer/scheduler.
     *
     * @param[in] node        Autolink node for action clients.
     * @param[in] muxer       Shared task muxer (may be null).
     * @param[in] scheduler   Work pool for background Execute (non-null).
     * @param[in] idempotency Optional command-id cache.
     */
    TeleopRelativeBackend(std::shared_ptr<autolink::Node> node,
                          TaskMuxer::SharedPtr muxer,
                          WorkScheduler* scheduler,
                          CommandIdempotencyCache* idempotency = nullptr);

    /**
     * @brief Install a predicate that reports Velocity GoalChannel busy.
     *
     * @param[in] check Returns true when Velocity session should block relative.
     */
    void SetVelocityBusyCheck(BusyCheck check) {
        velocity_busy_ = std::move(check);
    }

    /**
     * @brief Whether a relative action or velocity session is busy.
     *
     * @return true if mode_ != kNone or velocity_busy_ reports busy.
     */
    bool IsBusy() const;

    /**
     * @brief Start DriveOnHeading via ActionBackgroundInterface.
     *
     * @param[in] request  DriveOnHeadingRequest.
     * @param[in] callback Stream sink.
     * @return false if rejected before schedule.
     */
    bool HandleDriveOnHeading(
        const ::automsgs::rpcs::teleop::DriveOnHeadingRequest& request,
        StreamCallback callback);

    /**
     * @brief Start BackUp via ActionBackgroundInterface.
     *
     * @param[in] request  BackUpRequest.
     * @param[in] callback Stream sink.
     * @return false if rejected before schedule.
     */
    bool HandleBackUp(const ::automsgs::rpcs::teleop::BackUpRequest& request,
                      StreamCallback callback);

    /**
     * @brief Start Spin via ActionBackgroundInterface.
     *
     * @param[in] request  SpinRequest.
     * @param[in] callback Stream sink.
     * @return false if rejected before schedule.
     */
    bool HandleSpin(const ::automsgs::rpcs::teleop::SpinRequest& request,
                    StreamCallback callback);

    /**
     * @brief Cancel the active relative goal (optional goal_id filter).
     *
     * @param[in] goal_id Empty matches any; otherwise must match goal_id_.
     * @return            true if a cancel was issued or no-op success.
     */
    bool CancelGoal(const std::string& goal_id = {});

    /**
     * @brief Pause the active relative goal (best-effort).
     *
     * @param[in] goal_id Optional goal id filter.
     * @return            true if pause was accepted / no-op.
     */
    bool PauseGoal(const std::string& goal_id = {});

    /**
     * @brief Resume the active relative goal (best-effort).
     *
     * @param[in] goal_id Optional goal id filter.
     * @return            true if resume was accepted / no-op.
     */
    bool ResumeGoal(const std::string& goal_id = {});

    /**
     * @brief Snapshot of current teleop relative state.
     *
     * @return TeleopResponse reflecting mode_ / state_ / goal_id_.
     */
    ::automsgs::rpcs::teleop::TeleopResponse GetSnapshot() const;

private:
    /**
     * @brief Active relative-motion mode (at most one non-kNone).
     */
    enum class Mode {
        /** @brief No relative action running. */
        kNone,
        /** @brief DriveOnHeading in progress. */
        kDrive,
        /** @brief BackUp in progress. */
        kBackUp,
        /** @brief Spin in progress. */
        kSpin
    };

    /**
     * @brief BackgroundCommandSession specialized for TeleopResponse.
     */
    using Session =
        BackgroundCommandSession<::automsgs::rpcs::teleop::TeleopResponse>;

    /**
     * @brief Clear mode_ / handles / paused_ under mutex_ (caller holds lock).
     */
    void ClearLocked();

    /**
     * @brief Build a TeleopResponse with status overlay.
     *
     * @param[in] goal_id Goal id to stamp.
     * @param[in] state   TeleopState enum.
     * @param[in] ok      Success bit for status.
     * @param[in] detail  Optional detail text.
     * @return            Populated TeleopResponse.
     */
    ::automsgs::rpcs::teleop::TeleopResponse MakeResponse(
        const std::string& goal_id, ::automsgs::rpcs::teleop::TeleopState state,
        bool ok, const std::string& detail = {}) const;

    /**
     * @brief Shared Start path for Drive / BackUp / Spin traits.
     *
     * @tparam Traits RelativeActionBase leaf (DriveOnHeadingTraits, …).
     * @param[in,out] client      Action client for this mode.
     * @param[in]     request     Relative RPC request.
     * @param[in]     callback    Stream sink.
     * @param[in]     mode        Mode to set on accept.
     * @param[out]    handle_slot Slot that receives the accepted GoalHandle.
     * @return                    false if rejected before schedule.
     */
    template <typename Traits>
    bool StartRelativeAction(
        typename Traits::Client& client,
        const typename Traits::Request& request, StreamCallback callback,
        Mode mode,
        std::shared_ptr<typename Traits::Client::GoalHandle>* handle_slot);

    /**
     * @brief DriveOnHeading action client (SharedPtr).
     */
    DriveOnHeadingTraits::Client::SharedPtr drive_client_{nullptr};

    /**
     * @brief Active DriveOnHeading goal handle (null when idle).
     */
    std::shared_ptr<DriveOnHeadingTraits::Client::GoalHandle> drive_handle_{nullptr};

    /**
     * @brief BackUp action client (SharedPtr).
     */
    BackUpTraits::Client::SharedPtr backup_client_{nullptr};

    /**
     * @brief Active BackUp goal handle (null when idle).
     */
    std::shared_ptr<BackUpTraits::Client::GoalHandle> backup_handle_{nullptr};

    /**
     * @brief Spin action client (SharedPtr).
     */
    SpinTraits::Client::SharedPtr spin_client_{nullptr};

    /**
     * @brief Active Spin goal handle (null when idle).
     */
    std::shared_ptr<SpinTraits::Client::GoalHandle> spin_handle_{nullptr};

    /**
     * @brief Shared TaskMuxer for exclusive teleop slot acquire/release.
     */
    TaskMuxer::SharedPtr muxer_{nullptr};

    /**
     * @brief Background session coordinating stream emit / Execute.
     */
    Session session_;

    /**
     * @brief Optional Velocity-busy predicate installed by TeleopStub.
     */
    BusyCheck velocity_busy_;

    /**
     * @brief Protects mode_ / goal_id_ / paused_ / state_ / handles.
     */
    mutable std::mutex mutex_;

    /**
     * @brief Currently active relative mode (kNone when idle).
     */
    Mode mode_{Mode::kNone};

    /**
     * @brief goal_id of the active relative command (empty when idle).
     */
    std::string goal_id_;

    /**
     * @brief True after PauseGoal until ResumeGoal / ClearLocked.
     */
    bool paused_{false};

    /**
     * @brief Mirrored TeleopState for GetSnapshot.
     */
    ::automsgs::rpcs::teleop::TeleopState state_{
        ::automsgs::rpcs::teleop::TELEOP_STATE_IDLE};
};

}  // namespace teleop
}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
