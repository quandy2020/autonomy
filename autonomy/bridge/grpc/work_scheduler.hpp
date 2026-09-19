/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file work_scheduler.hpp
 * @brief Bridge background work over autolink::base::ThreadPool.
 *
 * Handler / async_grpc event threads must not block on Action result waits.
 * Domain stubs and BackgroundCommandSession push blocking work here.
 *
 * @par Invariants
 * - Handler / event threads must not block; use Schedule for Action waits.
 * - Schedule is fire-and-forget; use Enqueue when a future is required.
 * - Pool is owned here; Context / stubs hold non-owning pointers / SharedPtr
 * into the Server-owned WorkScheduler.
 */

#pragma once

#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <utility>

#include "autolink/base/thread_pool.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Schedules work on an autolink ThreadPool (bounded queue).
 *
 * Thin wrapper that owns the ThreadPool and exposes Schedule (void work) plus
 * Enqueue (future-returning). Rejects construction with non-positive
 * num_threads (implementation may throw / abort — callers pass options from
 * GrpcOptions).
 *
 * @par Threading
 * Schedule / Enqueue are thread-safe via the underlying pool.
 * Destructor joins workers; do not Schedule after the owning Server begins
 * teardown without synchronizing in-flight lambdas.
 *
 * @par Ownership
 * Server holds SharedPtr; Context stores the same SharedPtr;
 * Session / stubs typically keep raw WorkScheduler* or SharedPtr copies.
 */
class WorkScheduler
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(WorkScheduler)

    /**
     * @brief Unit of fire-and-forget work (no return value).
     */
    using WorkItem = std::function<void()>;

    /**
     * @brief Construct and start the underlying ThreadPool.
     *
     * @param[in] num_threads Worker count (must be > 0).
     * @param[in] max_tasks   Bounded queue capacity (default 1024); excess
     *                       Schedule behavior follows ThreadPool policy.
     */
    explicit WorkScheduler(int num_threads, std::size_t max_tasks = 1024);

    /**
     * @brief Join workers and destroy the pool.
     *
     * @warning In-flight WorkItems must not touch destroyed Context / stubs;
     * Shutdown the gRPC server and drain tasks first.
     */
    ~WorkScheduler();

    WorkScheduler(const WorkScheduler&) = delete;
    WorkScheduler& operator=(const WorkScheduler&) = delete;

    /**
     * @brief Fire-and-forget enqueue.
     *
     * @param[in] work Callable run on a worker thread; moved into the queue.
     *
     * @note Exceptions escaping @p work are the caller's responsibility unless
     * the concrete WorkItem catches (BackgroundCommandSession does).
     */
    void Schedule(WorkItem work);

    /**
     * @brief Enqueue and return a future for the result.
     *
     * Perfect-forwards to ThreadPool::Enqueue. Prefer Schedule for Session
     * Execute bodies that stream via callbacks instead of returning values.
     *
     * @tparam F    Callable type.
     * @tparam Args Argument types forwarded to @p f.
     * @param[in] f    Callable to run on a worker.
     * @param[in] args Arguments bound to @p f.
     * @return         std::future for the invoke result.
     */
    template <typename F, typename... Args>
    auto Enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type> {
        return pool_->Enqueue(std::forward<F>(f), std::forward<Args>(args)...);
    }

    /**
     * @brief Configured worker count.
     *
     * @return num_threads passed to the constructor.
     */
    int num_threads() const { return num_threads_; }

private:
    /**
     * @brief Worker count passed to the constructor (must be > 0).
     */
    int num_threads_{0};

    /**
     * @brief Owned Autolink ThreadPool that runs Schedule / Enqueue work.
     */
    std::unique_ptr<::autolink::base::ThreadPool> pool_{nullptr};
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
