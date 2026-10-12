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

#include <bolza/core/thread_support/set_thread_priority.hpp>
#include <bolza/core/thread_support/thread_pool.hpp>

namespace bolza {

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::ThreadPool(size_t nThreads, int priority) {
  workerThreads_.reserve(nThreads);
  for (size_t i = 0; i < nThreads; i++) {
    workerThreads_.emplace_back(&ThreadPool::Worker, this, i);
    SetThreadPriority(priority, workerThreads_.back());
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::~ThreadPool() {
  {  // set exit flag, wake up threads and join
    std::lock_guard<std::mutex> lock(taskQueueLock_);
    stop_ = true;
  }
  taskQueueCondition_.notify_all();
  for (auto& thread : workerThreads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::Worker(int workerIndex) {
  while (true) {
    std::unique_ptr<ThreadPool::TaskBase> taskPtr;
    {
      std::unique_lock<std::mutex> lock(taskQueueLock_);
      taskQueueCondition_.wait(lock, [this] { return !taskQueue_.empty() || stop_; });

      // exit condition
      if (stop_) {
        break;
      }

      // pop the first task
      if (!taskQueue_.empty()) {
        taskPtr = std::move(taskQueue_.front());
        taskQueue_.pop();
      }
    }

    if (taskPtr) {
      taskPtr->operator()(workerIndex);
    }
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::RunTask(std::unique_ptr<TaskBase> taskPtr) {
  {
    std::lock_guard<std::mutex> lock(taskQueueLock_);
    taskQueue_.push(std::move(taskPtr));
  }
  taskQueueCondition_.notify_one();
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::RunParallel(std::function<void(int)> taskFunction, int N) {
  // Launch tasks in helper threads
  std::vector<std::future<void>> futures;
  if (N > 1) {
    const int numHelpers = N - 1;
    futures.reserve(numHelpers);
    for (int i = 0; i < numHelpers; ++i) {
      futures.emplace_back(Run(taskFunction));
    }
  }

  // Execute one instance in this thread.
  const auto workerId = static_cast<int>(NumThreads());  // threadpool workers use ID 0 -> nThreads - 1
  taskFunction(workerId);

  // Wait for helpers to finish.
  for (auto&& fut : futures) {
    fut.get();
  }
}

}  // namespace bolza
