/* Copyright 2026 The Openbot Authors. */
/** @file reader_retirement.cpp
 * @brief Deferred reader teardown with explicit completion and dependency lifetime.
 */
#include "autonomy/perception/observation/reader_retirement.hpp"

#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace autonomy::perception {
/** @brief Shared operation state outliving the requesting source if needed. */
struct ReaderRetirement::State {
    explicit State(std::function<bool()> operation)
        : cleanup(std::move(operation)), completion(promise.get_future().share()) {}
    mutable std::mutex mutex;  ///< Guards request state and worker identity.
    std::condition_variable changed;  ///< Wakes the worker once shutdown is requested.
    bool requested = false;  ///< Monotonic cleanup request.
    bool startup_failed = false;  ///< Abort a worker before subscriptions if detach fails.
    std::thread::id worker_id;  ///< Detects a cleanup callback attempting to wait on itself.
    std::function<bool()> cleanup;  ///< Pins actual readers/node until cleanup completes.
    std::promise<bool> promise;  ///< Fulfilled after releasing cleanup captures.
    std::shared_future<bool> completion;  ///< Stable result shared with external owners.
};

ReaderRetirement::ReaderRetirement(std::function<bool()> cleanup) {
    if (!cleanup) throw std::invalid_argument("reader retirement needs a cleanup operation");
    state_ = std::make_shared<State>(std::move(cleanup));
    std::thread worker([state = state_] {
        std::function<bool()> operation;
        {
            std::unique_lock<std::mutex> lock(state->mutex);
            state->worker_id = std::this_thread::get_id();
            state->changed.wait(lock, [&] { return state->requested || state->startup_failed; });
            if (state->startup_failed) {
                state->cleanup = {};
                state->promise.set_value(false);
                return;
            }
            operation = std::move(state->cleanup);
        }
        bool succeeded = false;
        try { succeeded = operation(); } catch (...) {}
        // Reader/node pins must be released before an owner sees completion.
        operation = {};
        state->promise.set_value(succeeded);
    });
    // Lifetime is tracked by the shared state and completion handle, not by a
    // joinable thread member that might be destroyed on a native callback.
    try {
        worker.detach();
    } catch (...) {
        // No native Reader has been created yet. Wake the waiting worker without
        // invoking caller cleanup, then join before propagating startup failure.
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->startup_failed = true;
        }
        state_->changed.notify_one();
        if (worker.joinable()) worker.join();
        throw;
    }
}

ReaderRetirement::~ReaderRetirement() { Request(); }
void ReaderRetirement::Request() noexcept {
    { std::lock_guard<std::mutex> lock(state_->mutex); state_->requested = true; }
    state_->changed.notify_one();
}
bool ReaderRetirement::WaitFor(std::chrono::milliseconds timeout) const {
    if (timeout.count() < 0) return false;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->worker_id == std::this_thread::get_id()) return false;
    }
    return state_->completion.wait_for(timeout) == std::future_status::ready && state_->completion.get();
}
std::shared_future<bool> ReaderRetirement::Completion() const { return state_->completion; }
ReaderRetirementSnapshot ReaderRetirement::Snapshot() const {
    ReaderRetirementSnapshot result;
    { std::lock_guard<std::mutex> lock(state_->mutex); result.requested = state_->requested; }
    result.finished = state_->completion.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
    if (result.finished) result.succeeded = state_->completion.get();
    return result;
}
}  // namespace autonomy::perception
