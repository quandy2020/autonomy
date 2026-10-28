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
 * @file thread_pool.hpp
 * @brief Fixed-size worker pool and scheduling interface.
 */

#ifndef ATLAS_COMMON_THREAD_POOL_HPP_
#define ATLAS_COMMON_THREAD_POOL_HPP_

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace atlas {
namespace common {

class Task;

/** @brief Interface used by modules that schedule independent work items. */
class ThreadPoolInterface {
 public:
  virtual ~ThreadPoolInterface() = default;

  /**
   * @brief Schedule a task for asynchronous execution.
   * @throws std::logic_error if the scheduler has shut down.
   */
  virtual void Schedule(std::function<void()> task) = 0;

  /** Schedule a dependency-aware task and return a completion handle. */
  virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

  /**
   * @brief Submit a callable and return its future result.
   * @throws std::logic_error if the scheduler has shut down.
   */
  template <typename TaskFunction>
  auto Submit(TaskFunction&& task_function)
      -> std::future<std::invoke_result_t<TaskFunction>> {
    using ResultType = std::invoke_result_t<TaskFunction>;
    auto packaged_task = std::make_shared<std::packaged_task<ResultType()>>(
        std::forward<TaskFunction>(task_function));
    std::future<ResultType> task_result = packaged_task->get_future();
    Schedule([packaged_task] { (*packaged_task)(); });
    return task_result;
  }

 protected:
  void Execute(Task* task);
  void SetThreadPool(Task* task);

 private:
  friend class Task;
  virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

/**
 * @brief Fixed-size worker pool that drains queued work during shutdown.
 *
 * Tasks submitted through @ref Submit return a future that carries the task's
 * result or exception. Tasks submitted directly through @ref Schedule must
 * handle their own exceptions; an uncaught exception is contained so the
 * worker remains available.
 */
class ThreadPool final : public ThreadPoolInterface {
 public:
  /** @brief Start the requested number of workers; zero is clamped to one. */
  explicit ThreadPool(std::size_t worker_count);
  ~ThreadPool() override;

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  /** @brief Schedule a task; throws when called after shutdown begins. */
  void Schedule(std::function<void()> task) override;
  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override;

  /**
   * @brief Stop accepting work, drain queued tasks, and join all workers.
   * Must be called outside this pool's workers, including destruction. Worker
   * tasks must not synchronously wait for work queued in the same pool.
   */
  void Shutdown();

 private:
  /** @brief Wait for work, execute queued tasks, and exit after draining. */
  void WorkerLoop();
  void NotifyDependenciesCompleted(Task* task) override;

  std::vector<std::thread> worker_threads_;
  std::queue<std::function<void()>> pending_tasks_;
  std::unordered_map<Task*, std::shared_ptr<Task>> waiting_tasks_;
  std::mutex queue_mutex_;
  std::mutex shutdown_mutex_;
  std::condition_variable task_available_condition_;
  bool shutdown_requested_ = false;
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_THREAD_POOL_HPP_
