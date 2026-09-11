/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared stream-session state for multi-channel command stubs.
 */

#pragma once

#include <functional>
#include <mutex>
#include <utility>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Mutex-protected stream session (callback + last request + active).
 *
 * @tparam RequestT Last accepted request type.
 * @tparam ResponseT Stream response type.
 */
template <typename RequestT, typename ResponseT>
class StreamSessionState
{
public:
    using Request = RequestT;
    using Response = ResponseT;
    using StreamCallback = std::function<void(const Response&)>;

    /**
     * @brief Bind the stream sink and remember the request.
     * @param[in] request Last request.
     * @param[in] callback Stream sink.
     */
    void BindStream(const Request& request, StreamCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_request_ = request;
        stream_callback_ = std::move(callback);
    }

    /**
     * @brief Mark whether the session is active.
     * @param[in] active Session flag.
     */
    void SetSessionActive(bool active) {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = active;
    }

    /** @brief Check whether the session is active. */
    bool CheckSessionActive() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_active_;
    }

    /** @brief Clear callback and active flag. */
    void ClearSession() {
        std::lock_guard<std::mutex> lock(mutex_);
        session_active_ = false;
        stream_callback_ = nullptr;
    }

    /**
     * @brief Emit a response if a callback is bound.
     * @param[in] response Payload to send.
     * @return true if a callback was invoked.
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
     * @param[in] clear_session When true, deactivate and drop the callback.
     * @param[out] request Copied last request.
     * @return Bound callback, or empty.
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

    /** @brief Copy the last request under lock. */
    Request GetLastRequest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_request_;
    }

    /** @brief Access the mutex for compound updates by the owner stub. */
    std::mutex& mutex() const { return mutex_; }

    bool session_active_unlocked() const { return session_active_; }
    StreamCallback& stream_callback_unlocked() { return stream_callback_; }
    const StreamCallback& stream_callback_unlocked() const {
        return stream_callback_;
    }
    Request& last_request_unlocked() { return last_request_; }
    const Request& last_request_unlocked() const { return last_request_; }

private:
    mutable std::mutex mutex_;
    bool session_active_{false};
    Request last_request_{};
    StreamCallback stream_callback_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
