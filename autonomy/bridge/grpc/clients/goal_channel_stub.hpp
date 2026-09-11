/*
 * Copyright 2026 The Openbot Authors
 *
 * Traits-based session over a goal Writer and feedback Reader.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace detail {

template <typename Traits, typename = void>
struct HasCheckEmitFeedback : std::false_type {};

template <typename Traits>
struct HasCheckEmitFeedback<
    Traits,
    std::void_t<decltype(Traits::CheckEmitFeedback(
        std::declval<const typename Traits::Feedback&>(), true))>>
    : std::true_type {};

}  // namespace detail

/**
 * @brief Session helper that publishes goals and mirrors feedback to a stream.
 *
 * @tparam Traits Compile-time policy. Required members:
 *   - nested types `Goal`, `Feedback`, `Request`, `Response`
 *   - `kTaskType`, `kGoalChannel`, `kFeedbackChannel`
 *   - `ConvertToGoal`, `ConvertFromFeedback`, `MakeResponse`,
 *     `CheckTerminalStatus`
 *
 * Optional:
 *   - `CheckEmitFeedback(feedback, session_active)` — when absent, feedback is
 *     emitted only while the session is active.
 */
template <typename Traits>
class GoalChannelStub
{
public:
    using Goal = typename Traits::Goal;
    using Feedback = typename Traits::Feedback;
    using Request = typename Traits::Request;
    using Response = typename Traits::Response;
    using StreamCallback = std::function<void(const Response&)>;
    using FeedbackHook = std::function<void(const Feedback&)>;

    /**
     * @brief Create writers/readers for the Traits channels.
     * @param[in] node Autolink node used for pub/sub.
     * @param[in] muxer Shared task muxer (may be null).
     */
    GoalChannelStub(std::shared_ptr<autolink::Node> node,
                    std::shared_ptr<TaskMuxer> muxer)
        : node_(std::move(node)), muxer_(std::move(muxer)) {
        if (!node_) {
            return;
        }
        goal_writer_ = node_->CreateWriter<Goal>(Traits::kGoalChannel);
        if (!goal_writer_) {
            AERROR << "GoalChannelStub: writer failed on "
                   << Traits::kGoalChannel;
        }
        GoalChannelStub* self = this;
        feedback_reader_ = node_->CreateReader<Feedback>(
            Traits::kFeedbackChannel,
            [self](const std::shared_ptr<Feedback>& feedback) {
                self->HandleFeedback(feedback);
            });
        if (!feedback_reader_) {
            AWARN << "GoalChannelStub: feedback reader unavailable on "
                  << Traits::kFeedbackChannel;
        }
    }

    /** @brief Check whether the goal writer is available. */
    bool CheckWriterReady() const { return static_cast<bool>(goal_writer_); }

    /**
     * @brief Publish a goal message.
     * @param[in] goal Goal payload.
     * @return true if write succeeded.
     */
    bool WriteGoal(const Goal& goal) {
        return goal_writer_ && goal_writer_->Write(goal);
    }

    /** @brief Check whether the muxer reports emergency-stop. */
    bool CheckEstopActive() const { return muxer_ && muxer_->CheckEstopActive(); }

    /**
     * @brief Try to acquire the muxer slot for this Traits task type.
     * @param[in] request Request used for `cmd_id` / `client_id`.
     * @return true if acquired (or muxer is null).
     */
    bool TryAcquireTask(const Request& request) {
        if (!muxer_) {
            return true;
        }
        const std::string cmd_id =
            request.has_header() ? request.header().cmd_id() : "";
        const std::string client_id =
            request.has_header() ? request.header().client_id() : "";
        return muxer_->TryAcquire(Traits::kTaskType, cmd_id, client_id);
    }

    /** @brief Release the muxer slot for this Traits task type. */
    void ReleaseTaskSlot() {
        if (muxer_) {
            muxer_->Release(Traits::kTaskType);
        }
    }

    /**
     * @brief Bind the gRPC stream callback and remember the last request.
     * @param[in] request Last accepted request (for feedback conversion).
     * @param[in] callback Stream sink.
     */
    void BindStream(const Request& request, StreamCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        stream_callback_ = std::move(callback);
        last_request_ = request;
    }

    /**
     * @brief Install a side-effect hook invoked before feedback conversion.
     * @param[in] hook Optional hook (e.g. MapStub map-name cache).
     */
    void SetFeedbackHook(FeedbackHook hook) {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback_hook_ = std::move(hook);
    }

    /**
     * @brief Mark whether a feedback session is active.
     * @param[in] active Session flag.
     */
    void SetSessionActive(bool active) {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = active;
    }

    /**
     * @brief Clear stream state and optionally release the muxer slot.
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

    /** @brief Check whether a feedback session is active. */
    bool CheckSessionActive() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_active_;
    }

    /**
     * @brief Build a bridge response via Traits.
     * @see Traits::MakeResponse
     */
    Response MakeResponse(const Request& request, bool success, bool final,
                          const std::string& message = "") const {
        return Traits::MakeResponse(request, success, final, message);
    }

    /**
     * @brief Simple acquire → write → accepted-ack path (Map-like).
     * @param[in] request Bridge command request.
     * @param[in] stream_callback Stream sink.
     * @return false if rejected immediately.
     */
    bool HandleCommand(const Request& request, StreamCallback stream_callback) {
        if (!stream_callback) {
            return false;
        }
        if (!goal_writer_) {
            stream_callback(Traits::MakeResponse(
                request, false, true, "goal writer unavailable"));
            return false;
        }
        if (CheckEstopActive()) {
            stream_callback(Traits::MakeResponse(request, false, true,
                                                 "emergency stop active"));
            return false;
        }

        BindStream(request, stream_callback);

        if (!TryAcquireTask(request)) {
            StreamCallback callback;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                callback = stream_callback_;
                stream_callback_ = nullptr;
            }
            if (callback) {
                callback(Traits::MakeResponse(request, false, true,
                                        "another task is already active"));
            }
            return false;
        }

        if (!WriteGoal(Traits::ConvertToGoal(request))) {
            ClearSession(true);
            stream_callback(Traits::MakeResponse(
                request, false, true, "failed to publish goal"));
            return false;
        }

        SetSessionActive(true);
        stream_callback(Traits::MakeResponse(request, true, false, ""));
        return true;
    }

    /**
     * @brief Publish a cancel goal and clear the session.
     * @param[in] cancel_request Cancel request converted via Traits.
     * @param[in] stream_callback Optional terminal ack sink.
     */
    void CancelActiveSession(const Request& cancel_request,
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

    /** @brief Clear the session and release the muxer slot. */
    void CancelActiveSession() { ClearSession(true); }

private:
    static bool CheckShouldEmit(const Feedback& feedback, bool session_active) {
        if constexpr (detail::HasCheckEmitFeedback<Traits>::value) {
            return Traits::CheckEmitFeedback(feedback, session_active);
        }
        return session_active;
    }

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
            if (!CheckShouldEmit(*feedback, session_active_)) {
                return;
            }
            if (hook) {
                hook(*feedback);
            }
            response = Traits::ConvertFromFeedback(*feedback, last_request_);
            callback = stream_callback_;
            if (Traits::CheckTerminalStatus(*feedback)) {
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

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<TaskMuxer> muxer_;

    std::shared_ptr<autolink::Writer<Goal>> goal_writer_;
    std::shared_ptr<autolink::Reader<Feedback>> feedback_reader_;

    mutable std::mutex mutex_;
    bool session_active_{false};
    Request last_request_;
    StreamCallback stream_callback_;
    FeedbackHook feedback_hook_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
