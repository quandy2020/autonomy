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
 * @file slam_scheduler.hpp
 * @brief SLAM background task scheduler backed by autolink::base::ThreadPool.
 *
 * Tracking stays on the caller thread; LocalMapping / LoopClosing run as
 * long-lived pool tasks and block on ThreadSafeQueue for keyframes
 * (prefer pool size ≥ 2).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_SCHEDULER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_SCHEDULER_HPP_

#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <utility>

#include "autolink/base/thread_pool.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::SlamScheduler
 * @brief Owns a ThreadPool for LocalMapping / LoopClosing and similar workers.
 *
 * Typical use: `SlamSystem::StartBackendWorkers` starts long loops via
 * `Schedule` / `Enqueue`; after `RequestFinish`, wait on the futures.
 */
class SlamScheduler {
public:
    using WorkItem = std::function<void()>;  ///< Void callable task

    /**
     * @brief Construct scheduler and create the thread pool.
     * @param num_threads Worker count (clamped to at least 2 in the implementation).
     * @param max_tasks Cap on pending tasks in the pool queue.
     */
    explicit SlamScheduler(std::size_t num_threads = 3,
                           std::size_t max_tasks = 256);
    ~SlamScheduler();

    SlamScheduler(const SlamScheduler&) = delete;
    SlamScheduler& operator=(const SlamScheduler&) = delete;

    /**
     * @brief Submit a fire-and-forget task (future discarded).
     * @param work Void callable.
     */
    void Schedule(WorkItem work);

    /**
     * @brief Submit a task that may return a result.
     * @tparam F Callable type.
     * @tparam Args Argument types.
     * @param f Callable.
     * @param args Arguments forwarded to `f`.
     * @return `std::future` matching the return type of `f`.
     */
    template <typename F, typename... Args>
    auto Enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type> {
        return pool_->Enqueue(std::forward<F>(f), std::forward<Args>(args)...);
    }

    /**
     * @brief Number of worker threads in the pool.
     * @return Thread count.
     */
    std::size_t num_threads() const { return num_threads_; }

    /**
     * @brief Access the underlying ThreadPool (debug / advanced enqueue).
     * @return Non-owning pointer; invalid after this object is destroyed.
     */
    ::autolink::base::ThreadPool* mutable_pool() {
        return pool_.get();
    }

private:
    std::size_t num_threads_ = 0;  ///< Actual thread count in use
    std::unique_ptr<::autolink::base::ThreadPool> pool_;  ///< Task pool
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_SCHEDULER_HPP_
