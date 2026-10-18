/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file relocalization_thread_pool.hpp
 * @brief Isolated worker pool for BnB (bounded queue, cancellable).
 */

#ifndef ARGUS_RELOCALIZATION_RELOCALIZATION_THREAD_POOL_HPP_
#define ARGUS_RELOCALIZATION_RELOCALIZATION_THREAD_POOL_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "argus/relocalization/relocalization_pool_config.hpp"

namespace argus {
namespace relocalization {

/**
 * @brief Dedicated pool — not shared with ESIKF / loop closure.
 *
 * Submit is non-blocking w.r.t. search work; never call Search on the caller.
 */
class RelocalizationThreadPool
{
public:
    using Task = std::function<void(std::atomic<bool>& cancel_flag)>;

    /**
     * @brief Construct pool (call Start before Submit).
     * @param config Worker count, queue bounds, and overflow policy.
     */
    explicit RelocalizationThreadPool(RelocalizationPoolConfig config = {});
    ~RelocalizationThreadPool();

    RelocalizationThreadPool(const RelocalizationThreadPool&) = delete;
    RelocalizationThreadPool& operator=(const RelocalizationThreadPool&) =
        delete;

    /** @brief Spawn worker threads. */
    void Start();
    /** @brief Stop workers and join threads. */
    void Stop();

    /**
     * @brief Enqueue a task. Returns false if rejected (OnFull::kReject).
     *        When dropping oldest, returns true after drop.
     * @param task Callable receiving a per-task cancel flag.
     * @return false when OnFull::kReject and queue is full; otherwise true.
     */
    bool Submit(Task task);

    /** @brief Request cancel on running tasks and clear the pending queue. */
    void CancelAll();

    /** @return Pending task count (approximate under contention). */
    std::size_t queue_size() const;
    /** @return Tasks dropped due to queue overflow. */
    int dropped_count() const {
        return dropped_count_.load();
    }
    /** @return Tasks completed since construction. */
    int completed_count() const {
        return completed_count_.load();
    }
    /** @return True between Start() and Stop(). */
    bool running() const {
        return running_.load();
    }

private:
    void WorkerLoop(int worker_index);

    RelocalizationPoolConfig config_;
    std::vector<std::thread> workers_;
    /** @brief Per-worker cancel flags (atomics are not vector-movable). */
    std::vector<std::unique_ptr<std::atomic<bool>>> cancel_flags_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Task> queue_;
    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_all_{false};
    std::atomic<int> dropped_count_{0};
    std::atomic<int> completed_count_{0};
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_RELOCALIZATION_THREAD_POOL_HPP_
