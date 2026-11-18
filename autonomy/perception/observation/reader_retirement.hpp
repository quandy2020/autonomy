/* Copyright 2026 The Openbot Authors. */
/** @file reader_retirement.hpp
 * @brief Request reader cleanup without executing teardown on a callback thread.
 */
#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>

namespace autonomy::perception {

/** @brief Observable cleanup state; a timeout is not a completed shutdown. */
struct ReaderRetirementSnapshot {
    bool requested = false;  ///< Entry revocation has requested cleanup.
    bool finished = false;  ///< Cleanup and release of its captured resources ended.
    bool succeeded = false;  ///< Cleanup returned true without throwing.
};

/** @brief Own one deferred cleanup operation and pin its dependencies until done.
 * @details The worker is created before native readers exist. Request and
 * destruction only notify it; callbacks never execute or wait for teardown.
 * The completion future allows an external owner to audit draining. A blocked
 * native teardown remains outstanding, rather than being declared complete.
 */
class ReaderRetirement {
public:
    /** @brief Start a waiting worker with all cleanup dependencies captured.
     * @throws std::invalid_argument If cleanup is empty.
     * @throws std::system_error If a worker cannot be created, before subscription.
     */
    explicit ReaderRetirement(std::function<bool()> cleanup);
    /** @brief Request cleanup; the worker retains its state after this object dies. */
    ~ReaderRetirement();
    ReaderRetirement(const ReaderRetirement&) = delete;
    ReaderRetirement& operator=(const ReaderRetirement&) = delete;
    /** @brief Request exactly one cleanup without waiting for any callback. */
    void Request() noexcept;
    /** @brief Wait externally for successful cleanup within the given duration.
     * @return False on timeout, failure, negative duration or worker-thread reentry.
     */
    bool WaitFor(std::chrono::milliseconds timeout) const;
    /** @brief Copy a stable completion handle for owner shutdown accounting. */
    std::shared_future<bool> Completion() const;
    /** @brief Read requested, finished and success independently of native locks. */
    ReaderRetirementSnapshot Snapshot() const;
private:
    struct State;  ///< Shared synchronization, cleanup pin and completion promise.
    std::shared_ptr<State> state_;  ///< Worker and external handles retain operation state.
};
}  // namespace autonomy::perception
