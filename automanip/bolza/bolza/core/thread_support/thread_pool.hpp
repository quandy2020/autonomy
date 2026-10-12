/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file thread_pool.hpp
 * @brief Thread pool class to execute tasks on multiple threads.
 */


#include <condition_variable>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace bolza {

/**
 * Thread pool class to execute tasks on multiple threads.
 */
class ThreadPool {
 public:
  /**
   * Constructor
   *
   * @param [in] nThreads: Number of threads to launch in the pool
   * @param [in] priority: The worker thread priority
   */
  explicit ThreadPool(size_t nThreads = 1, int priority = 0);

  /**
   * Destructor
   */
  ~ThreadPool();

  /**
   * Run a task in another thread
   *
   * @tparam Functor: The task function
   * @param [in] taskFunction: The task function to run in the pool. It takes a thread worker index argument (between 0 and nThreads - 1),
                               which can be used to index designated thread resources.
   * @return future object with taskFunction return value
   */
  template <typename Functor>
  /**
   * @brief Functor.
   * @param [in] taskFunction: Task Function.
   * @return Result value.
   */
  std::future<typename std::result_of<Functor(int)>::type> Run(Functor taskFunction);

  /**
   * Helper function to run a task N times parallel with the help of the pool.
   * - 1 task will run in the calling thread with ID = nThreads.
   * - N-1 tasks will run on the threadpool with ID in [0, nThreads-1].
   *
   * @note This is a blocking operation, returns when all tasks are completed.
   * @warning Calling RunParallel(task, nThreads) does not guarantee that each task will be executed with a different workerIndex.
   *
   * @param [in] taskFunction: task function to run in the pool.
   * @param [in] N: number of times to run taskFunction in parallel.
   */
  void RunParallel(std::function<void(int)> taskFunction, int N);

  /**
   * @brief Get the number of threads.
   * @return Result value.
   */
  size_t NumThreads() const { /**
  * @brief Size.
  * @return Result value.
  */
 return workerThreads_.size(); }

 private:
  struct TaskBase;

  template <typename Functor>
  struct Task;

  /**
   * Thread worker loop
   *
   * @param [in] workerIndex: worker thread index
   */
  void Worker(int workerIndex);

  /**
   * Run a task asynchronously in another thread
   *
   * @param [in] taskPtr: task object
   */
  void RunTask(std::unique_ptr<TaskBase> taskPtr);

  bool stop_{false};  //!< flag telling all threads to stop, protected by taskQueueLock_

  std::queue<std::unique_ptr<TaskBase>> taskQueue_;  // protected by taskQueueLock_
  std::condition_variable taskQueueCondition_;
  std::mutex taskQueueLock_;

  std::vector<std::thread> workerThreads_;
};

/**
 * Task callback interface class.
 */
struct ThreadPool::TaskBase {
  /**
   * @brief Constructs a TaskBase.
   */
  TaskBase() = default;
  /**
   * @brief Destructor.
   */
  virtual ~TaskBase() = default;
  virtual void operator()(int workerIndex) = 0;
};

/**
 * Task callback for a specific function return type.
 *
 * @tparam Functor: Type of callable task object (functor).
 */
template <typename Functor>
struct ThreadPool::Task final : public ThreadPool::TaskBase {
  /**
   * @brief Constructs a Task.
   */
  explicit Task(Functor taskFunction) : packagedTask(std::move(taskFunction)) {}
  /**
   * @brief Destructor.
   */
  ~Task() override = default;
  void operator()(int workerIndex) override { packagedTask(workerIndex); }

  using ReturnType = typename std::result_of<Functor(int)>::type;
  std::packaged_task<ReturnType(int)> packagedTask;
};

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
template <typename Functor>
/**
 * @brief Functor.
 * @param [in] taskFunction: Task Function.
 * @return Result value.
 */
std::future<typename std::result_of<Functor(int)>::type> ThreadPool::Run(Functor taskFunction) {
  /**
   * @brief Move.
   * @return Result value.
   */
  auto taskPtr = std::make_unique<Task<Functor>>(std::move(taskFunction));
  /**
   * @brief Get future.
   * @return Result value.
   */
  auto future = taskPtr->packagedTask.get_future();

  if (workerThreads_.empty()) {
    // run on main thread
    taskPtr->operator()(0);
  } else {
    /**
     * @brief Constructs a RunTask.
     */
    RunTask(std::move(taskPtr));
  }

  return future;
}

}  // namespace bolza
