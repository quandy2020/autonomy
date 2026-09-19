/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file goal_channel_stub.hpp
 * @brief Non-blocking goal/feedback session driven by a Traits policy.
 *
 * @details
 * Publishes goals on Traits::kGoalTopic and mirrors feedback from
 * Traits::kFeedbackTopic into a bound StreamCallback. Domain stubs embed
 * this type via GoalChannelCommandStub. Optional Traits::ShouldEmit /
 * RejectReason customize emit/reject without subclassing.
 *
 * @par Ownership
 * node_ and muxer_ are shared; writers and readers are bound for the stub
 * lifetime. Does not own DomainBundle.
 *
 * @par Threading
 * BindStream / SetSessionActive / ClearSession / IsActive share mutex_;
 * HandleFeedback copies the callback under lock then invokes outside.
 *
 * @par Invariants
 * - Feedback callbacks only emit while IsActive(); terminal feedback clears
 *   the session and releases the muxer slot.
 * - Optional Traits::ShouldEmit / RejectReason customize emit/reject without
 *   subclassing (detected via HasShouldEmit / HasRejectReason).
 * - Lifecycle Pause/Resume/Cancel go through GoalChannelCommandStub::WriteCommand.
 *
 * @see GoalChannelCommandStub
 * @see command_dispatch.hpp
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include "autolink/common/log.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/command_dispatch.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

namespace detail {

/**
 * @brief Detect optional Traits::ShouldEmit(feedback, session_active).
 *
 * @tparam Traits GoalChannel traits candidate.
 */
template <typename Traits, typename = void>
struct HasShouldEmit : std::false_type {
    AUTONOMY_SMART_PTR_DEFINITIONS(HasShouldEmit<Traits>)
};

/**
 * @brief Specialization when Traits exposes ShouldEmit.
 *
 * @tparam Traits GoalChannel traits with ShouldEmit.
 */
template <typename Traits>
struct HasShouldEmit<
    Traits,
    std::void_t<decltype(Traits::ShouldEmit(
        std::declval<const typename Traits::Feedback&>(), true))>>
    : std::true_type {
    AUTONOMY_SMART_PTR_DEFINITIONS(HasShouldEmit<Traits>)
};

/**
 * @brief Detect optional Traits::RejectReason(request, session_active).
 *
 * Primary: false when RejectReason is absent.
 *
 * @tparam Traits GoalChannel traits candidate.
 */
template <typename Traits, typename = void>
struct HasRejectReason : std::false_type {
    AUTONOMY_SMART_PTR_DEFINITIONS(HasRejectReason<Traits>)
};

/**
 * @brief Specialization when Traits exposes RejectReason.
 *
 * @tparam Traits GoalChannel traits with RejectReason.
 *
 * @note RejectReason returns message text, or nullopt to continue.
 */
template <typename Traits>
struct HasRejectReason<
    Traits,
    std::void_t<decltype(Traits::RejectReason(
        std::declval<const typename Traits::Request&>(), true))>>
    : std::true_type {
    AUTONOMY_SMART_PTR_DEFINITIONS(HasRejectReason<Traits>)
};

}  // namespace detail

/**
 * @brief Session helper that publishes goals and mirrors feedback to a stream.
 *
 * @tparam Traits Compile-time policy. Required members:
 * - nested types `Goal`, `Feedback`, `Request`, `Response`
 * - `kTaskType`, `kGoalTopic`, `kFeedbackTopic`
 * - `ConvertToGoal`, `ConvertFromFeedback`, `MakeResponse`,
 * `IsTerminal`
 *
 * Optional:
 * - `ShouldEmit(feedback, session_active)` — when absent, feedback is
 * emitted only while the session is active.
 * - `RejectReason(request, session_active)` — pre-write reject message.
 *
 * @warning Do not call Dispatch / CancelSession from inside a feedback
 * callback that still holds an outer lock shared with this stub.
 */
template <typename Traits>
class GoalChannelStub
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(GoalChannelStub<Traits>)

    /**
     * @brief Goal protobuf published on kGoalTopic.
     */
    using Goal = typename Traits::Goal;

    /**
     * @brief Feedback protobuf received on kFeedbackTopic.
     */
    using Feedback = typename Traits::Feedback;

    /**
     * @brief Rpc request type converted via Traits::ConvertToGoal.
     */
    using Request = typename Traits::Request;

    /**
     * @brief Rpc / stream response type from Traits converters.
     */
    using Response = typename Traits::Response;

    /**
     * @brief Stream sink invoked with Response frames (ACK / feedback / terminal).
     */
    using StreamCallback = std::function<void(const Response&)>;

    /**
     * @brief Optional side-effect hook before feedback conversion.
     */
    using FeedbackHook = std::function<void(const Feedback&)>;

    /**
     * @brief Create writers/readers for the Traits channels.
     *
     * @param[in] node  Autolink node used for pub/sub (may be null → inert).
     * @param[in] muxer Shared task muxer (may be null → no acquire/release).
     */
    GoalChannelStub(std::shared_ptr<autolink::Node> node,
                    TaskMuxer::SharedPtr muxer)
        : node_(std::move(node)), muxer_(std::move(muxer)) {
        if (!node_) {
            return;
        }
        goal_writer_ = node_->CreateWriter<Goal>(Traits::kGoalTopic);
        if (!goal_writer_) {
            AERROR << "GoalChannelStub: writer failed on "
                   << Traits::kGoalTopic;
        }
        GoalChannelStub* self = this;
        feedback_reader_ = node_->CreateReader<Feedback>(
            Traits::kFeedbackTopic,
            [self](const std::shared_ptr<Feedback>& feedback) {
                self->HandleFeedback(feedback);
            });
        if (!feedback_reader_) {
            AWARN << "GoalChannelStub: feedback reader unavailable on "
                  << Traits::kFeedbackTopic;
        }
    }

    /**
     * @brief Check whether the goal writer is available.
     *
     * @return true if CreateWriter succeeded.
     */
    bool CheckWriterReady() const { return static_cast<bool>(goal_writer_); }

    /**
     * @brief Publish a goal message.
     *
     * @param[in] goal Goal payload.
     * @return         true if write succeeded.
     */
    bool WriteGoal(const Goal& goal) {
        return goal_writer_ && goal_writer_->Write(goal);
    }

    /**
     * @brief Check whether the muxer reports emergency-stop.
     *
     * @return true if muxer is non-null and IsEstop().
     */
    bool IsEstop() const { return muxer_ && muxer_->IsEstop(); }

    /**
     * @brief Try to acquire the muxer slot for this Traits task type.
     *
     * @param[in] request Request used for `cmd_id` / `client_id`.
     * @return            true if acquired (or muxer is null).
     */
    bool TryAcquireTask(const Request& request) {
        if (!muxer_) {
            return true;
        }
        return muxer_->TryAcquire(Traits::kTaskType, request.goal_id(), "");
    }

    /**
     * @brief Release the muxer slot for this Traits task type.
     */
    void ReleaseTaskSlot() {
        if (muxer_) {
            muxer_->Release(Traits::kTaskType);
        }
    }

    /**
     * @brief Bind the gRPC stream callback and remember the last request.
     *
     * @param[in] request  Last accepted request (for feedback conversion).
     * @param[in] callback Stream sink.
     *
     * @note Thread-safe; replaces any prior bound callback.
     */
    void BindStream(const Request& request, StreamCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        stream_callback_ = std::move(callback);
        last_request_ = request;
    }

    /**
     * @brief Install a side-effect hook invoked before feedback conversion.
     *
     * @param[in] hook Optional hook (e.g. MappingStub map-name cache).
     */
    void SetFeedbackHook(FeedbackHook hook) {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback_hook_ = std::move(hook);
    }

    /**
     * @brief Mark whether a feedback session is active.
     *
     * @param[in] active Session flag.
     */
    void SetSessionActive(bool active) {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = active;
    }

    /**
     * @brief Clear stream state and optionally release the muxer slot.
     *
     * @param[in] release_muxer When true, call `ReleaseTaskSlot()`.
     */
    void ClearSession(bool release_muxer) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            session_active_ = false;
            stream_callback_ = nullptr;
        }
        if (release_muxer) {
            ReleaseTaskSlot();
        }
    }

    /**
     * @brief Check whether a feedback session is active.
     *
     * @return true while session_active_ is set under lock.
     */
    bool IsActive() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_active_;
    }

    /**
     * @brief Build a response via Traits.
     *
     * @param[in] request Source request.
     * @param[in] success Success bit for status.
     * @param[in] final   Whether this is a terminal frame.
     * @param[in] message Optional detail text.
     * @return            Traits::MakeResponse result.
     *
     * @see Traits::MakeResponse
     */
    Response MakeResponse(const Request& request, bool success, bool final,
                          const std::string& message = "") const {
        return Traits::MakeResponse(request, success, final, message);
    }

    /**
     * @brief Simple acquire → write → accepted-ack path (Map-like START).
     *
     * @param[in] request         Rpc command request.
     * @param[in] stream_callback Stream sink.
     * @return                    false if rejected immediately.
     */
    bool HandleStart(const Request& request, StreamCallback stream_callback) {
        return Dispatch(
            request, std::move(stream_callback),
            /*acquire=*/true,
            [&](const StreamCallback& emit) {
                SetSessionActive(true);
                emit(Traits::MakeResponse(request, true, false, ""));
                return true;
            });
    }

    /**
     * @brief Shared gate → optional acquire → BindStream → WriteGoal → @p after_write.
     *
     * @param[in] request         Rpc command request.
     * @param[in] stream_callback Stream sink.
     * @param[in] acquire         When true, TryAcquire before write.
     * @param[in] after_write     Invoked after successful WriteGoal; receives emit.
     * @return                    false if rejected before/at write.
     */
    template <typename AfterWriteFn>
    bool Dispatch(const Request& request,
                               StreamCallback stream_callback, bool acquire,
                               AfterWriteFn&& after_write) {
        if (!stream_callback) {
            return false;
        }
        if (!goal_writer_) {
            stream_callback(Traits::MakeResponse(
                request, false, true, "goal writer unavailable"));
            return false;
        }
        if (IsEstop()) {
            stream_callback(Traits::MakeResponse(request, false, true,
                                                 "emergency stop active"));
            return false;
        }
        if constexpr (detail::HasRejectReason<Traits>::value) {
            if (const auto message = Traits::RejectReason(
                    request, IsActive())) {
                stream_callback(
                    Traits::MakeResponse(request, false, true, *message));
                return false;
            }
        }
        if (acquire && !TryAcquireTask(request)) {
            stream_callback(Traits::MakeResponse(
                request, false, true, "another task is already active"));
            return false;
        }

        BindStream(request, stream_callback);
        if (!WriteGoal(Traits::ConvertToGoal(request))) {
            ClearSession(acquire);
            stream_callback(Traits::MakeResponse(
                request, false, true, "failed to publish goal"));
            return false;
        }
        return after_write(stream_callback);
    }

    /**
     * @brief Publish a cancel goal and clear the session.
     *
     * @param[in] cancel_request  Cancel request converted via Traits.
     * @param[in] stream_callback Optional terminal ack sink.
     */
    void CancelSession(const Request& cancel_request,
                             StreamCallback stream_callback) {
        if (!goal_writer_) {
            if (stream_callback) {
                stream_callback(Traits::MakeResponse(
                    cancel_request, false, true, "goal writer unavailable"));
            }
            return;
        }
        WriteGoal(Traits::ConvertToGoal(cancel_request));
        if (stream_callback) {
            stream_callback(
                Traits::MakeResponse(cancel_request, true, true, ""));
        }
        ClearSession(true);
    }

    /**
     * @brief Clear the session and release the muxer slot.
     */
    void CancelSession() { ClearSession(true); }

private:
    /**
     * @brief Choose whether to emit @p feedback given session_active.
     *
     * @details Uses Traits::ShouldEmit when present; otherwise emits only
     * while the session is active.
     *
     * @param[in] feedback       Inbound feedback frame.
     * @param[in] session_active Current session_active_ snapshot.
     * @return                   true when ConvertFromFeedback / emit should run.
     */
    static bool AllowEmit(const Feedback& feedback, bool session_active) {
        if constexpr (detail::HasShouldEmit<Traits>::value) {
            return Traits::ShouldEmit(feedback, session_active);
        }
        return session_active;
    }

    /**
     * @brief Autolink feedback reader callback: convert and emit / clear.
     *
     * @details Copies callback under mutex_, invokes outside the lock, then
     * ReleaseTaskSlot() when Traits::IsTerminal.
     *
     * @param[in] feedback Shared feedback message (ignored when null).
     */
    void HandleFeedback(const std::shared_ptr<Feedback>& feedback) {
        if (!feedback) {
            return;
        }
        StreamCallback callback;
        Response response;
        bool clear = false;
        FeedbackHook hook;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            hook = feedback_hook_;
            if (!stream_callback_) {
                return;
            }
            if (!AllowEmit(*feedback, session_active_)) {
                return;
            }
            if (hook) {
                hook(*feedback);
            }
            response = Traits::ConvertFromFeedback(*feedback, last_request_);
            callback = stream_callback_;
            if (Traits::IsTerminal(*feedback)) {
                clear = true;
                session_active_ = false;
                stream_callback_ = nullptr;
            }
        }
        callback(response);
        if (clear) {
            ReleaseTaskSlot();
        }
    }

    /**
     * @brief Autolink node used to create goal writer / feedback reader.
     *
     * @details Shared ownership; may be null → stub is inert (no pub/sub).
     */
    std::shared_ptr<autolink::Node> node_{nullptr};

    /**
     * @brief Shared TaskMuxer for TryAcquire / Release / IsEstop.
     *
     * @details May be null → acquire always succeeds and estop is false.
     */
    TaskMuxer::SharedPtr muxer_{nullptr};

    /**
     * @brief Writer for Traits::kGoalTopic (null if create failed).
     */
    std::shared_ptr<autolink::Writer<Goal>> goal_writer_{nullptr};

    /**
     * @brief Reader for Traits::kFeedbackTopic (null if create failed).
     */
    std::shared_ptr<autolink::Reader<Feedback>> feedback_reader_{nullptr};

    /**
     * @brief Protects session_active_ / last_request_ / callbacks / hook.
     */
    mutable std::mutex mutex_;

    /**
     * @brief True while a feedback session should emit converted frames.
     */
    bool session_active_{false};

    /**
     * @brief Last request bound via BindStream (for ConvertFromFeedback).
     */
    Request last_request_;

    /**
     * @brief Bound stream sink for ACK / feedback / terminal frames.
     */
    StreamCallback stream_callback_;

    /**
     * @brief Optional pre-conversion feedback side-effect (e.g. map-name cache).
     */
    FeedbackHook feedback_hook_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
