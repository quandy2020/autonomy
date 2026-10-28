#pragma once

#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "atlas/common/blocking_queue.hpp"

namespace autonomy::localization::atlas {

/**
 * @brief Fixed-size worker pool backed by a closeable blocking queue.
 * @note Destruction stops submission and joins every worker.
 */
class ThreadPool
{
public:
    /**
     * @param thread_count Number of workers; must be greater than zero.
     * @param queue_capacity Maximum pending tasks, or zero for unbounded.
     */
    explicit ThreadPool(std::size_t thread_count,
                        std::size_t queue_capacity = 0);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /**
     * @brief Submits a callable and returns its result future.
     * @throws std::runtime_error if the pool has already stopped.
     */
    template <typename Function, typename... Args>
    auto Submit(Function&& function, Args&&... args)
        -> std::future<std::invoke_result_t<Function, Args...>> {
        using Result = std::invoke_result_t<Function, Args...>;
        auto task = std::make_shared<std::packaged_task<Result()>>(std::bind(
            std::forward<Function>(function), std::forward<Args>(args)...));
        std::future<Result> future = task->get_future();
        if (!tasks_.Push([task] { (*task)(); })) {
            throw std::runtime_error(
                "cannot submit work to a stopped ThreadPool");
        }
        return future;
    }

    /** @brief Closes the task queue and joins all workers. */
    void Stop();
    /** @return Number of worker threads owned by the pool. */
    std::size_t thread_count() const {
        return workers_.size();
    }

private:
    void WorkerLoop();

    BlockingQueue<std::function<void()>> tasks_;
    std::vector<std::thread> workers_;
};

}  // namespace autonomy::localization::atlas
