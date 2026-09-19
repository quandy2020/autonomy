/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file stream_session_state.hpp
 * @brief Shared stream-session state for multi-channel command stubs.
 *
 * @details
 * Mutex-protected holder for a gRPC stream sink, the last accepted request,
 * and an active flag. Used when a stub needs compound updates under a single
 * lock (Unlocked accessors) without owning TaskMuxer semantics.
 *
 * @par Ownership
 * Owns callback / last request by value. Does not own TaskMuxer; ClearSession
 * never releases muxer slots (caller responsibility).
 *
 * @par Threading
 * Callback / last request / active flag share one mutex. EmitResponse copies
 * the callback under lock then invokes outside.
 *
 * @par Invariants
 * - Callback / last request / active flag share one mutex.
 * - Emit is no-op when inactive or callback empty.
 * - ClearSession drops callback; does not touch TaskMuxer (owner does).
 * - *Unlocked accessors require the caller to already hold GetMutex().
 */

#pragma once

#include <functional>
#include <mutex>
#include <utility>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Mutex-protected stream session (callback + last request + active).
 *
 * @tparam RequestT  Last accepted request type.
 * @tparam ResponseT Stream response type.
 *
 * @par Ownership
 * Value-owns request / callback; no muxer ownership.
 *
 * @par Threading
 * All locked APIs take mutex_; Unlocked APIs require the caller to hold it.
 *
 * @warning EmitResponse copies the callback under lock then invokes outside;
 * do not deadlock by calling back into the same state while holding
 * GetMutex().
 */
template <typename RequestT, typename ResponseT>
class StreamSessionState
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for StreamSessionState.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(StreamSessionState<RequestT, ResponseT>)

    /**
     * @brief Alias for the last-accepted request type.
     */
    using Request = RequestT;

    /**
     * @brief Alias for the stream response type.
     */
    using Response = ResponseT;

    /**
     * @brief Stream sink invoked with Response frames.
     */
    using StreamCallback = std::function<void(const Response&)>;

    /**
     * @brief Bind the stream sink and remember the request.
     *
     * @param[in] request  Last request.
     * @param[in] callback Stream sink.
     */
    void BindStream(const Request& request, StreamCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_request_ = request;
        stream_callback_ = std::move(callback);
    }

    /**
     * @brief Mark whether the session is active.
     *
     * @param[in] active Session flag.
     */
    void SetSessionActive(bool active) {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = active;
    }

    /**
     * @brief Check whether the session is active.
     *
     * @return true while session_active_ is set.
     */
    bool IsActive() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_active_;
    }

    /**
     * @brief Clear callback and active flag.
     *
     * @note Does not release TaskMuxer slots; the owning stub must do that.
     */
    void ClearSession() {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = false;
        stream_callback_ = nullptr;
    }

    /**
     * @brief Emit a response if a callback is bound.
     *
     * @param[in] response Payload to send.
     * @return             true if a callback was invoked.
     */
    bool EmitResponse(const Response& response) {
        StreamCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            callback = stream_callback_;
        }
        if (!callback) {
            return false;
        }
        callback(response);
        return true;
    }

    /**
     * @brief Take the current callback (optionally clearing the session).
     *
     * @param[in]  clear_session When true, deactivate and drop the callback.
     * @param[out] request       Copied last request (optional).
     * @return                   Bound callback, or empty.
     */
    StreamCallback TakeCallback(bool clear_session, Request* request = nullptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (request) {
            *request = last_request_;
        }
        StreamCallback callback = stream_callback_;
        if (clear_session) {
            session_active_ = false;
            stream_callback_ = nullptr;
        }
        return callback;
    }

    /**
     * @brief Copy the last request under lock.
     *
     * @return Copy of last_request_.
     */
    Request GetLastRequest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_request_;
    }

    /**
     * @brief Access the mutex for compound updates by the owner stub.
     *
     * @return Reference to the shared session mutex.
     */
    std::mutex& GetMutex() const { return mutex_; }

    /**
     * @brief Session-active flag; caller must already hold mutex.
     *
     * @return session_active_ without locking.
     */
    bool IsActiveUnlocked() const { return session_active_; }

    /**
     * @brief Mutable stream callback; caller must already hold mutex.
     *
     * @return Reference to stream_callback_.
     */
    StreamCallback& GetStreamCallbackUnlocked() { return stream_callback_; }

    /**
     * @brief Const stream callback; caller must already hold mutex.
     *
     * @return Const reference to stream_callback_.
     */
    const StreamCallback& GetStreamCallbackUnlocked() const {
        return stream_callback_;
    }

    /**
     * @brief Mutable last request; caller must already hold mutex.
     *
     * @return Reference to last_request_.
     */
    Request& GetLastRequestUnlocked() { return last_request_; }

    /**
     * @brief Const last request; caller must already hold mutex.
     *
     * @return Const reference to last_request_.
     */
    const Request& GetLastRequestUnlocked() const { return last_request_; }

private:
    /**
     * @brief Protects session_active_ / last_request_ / stream_callback_.
     */
    mutable std::mutex mutex_;

    /**
     * @brief True while a feedback / stream session is considered active.
     */
    bool session_active_{false};

    /**
     * @brief Last request accepted via BindStream (for feedback conversion).
     */
    Request last_request_{};

    /**
     * @brief Bound gRPC / bridge stream sink (may be empty).
     */
    StreamCallback stream_callback_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
