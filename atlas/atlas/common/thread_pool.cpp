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

#include "common/thread_pool.hpp"

#include <stdexcept>

namespace atlas {
namespace common {

ThreadPool::ThreadPool(std::size_t worker_count) {
  if (worker_count == 0) {
    worker_count = 1;
  }
  worker_threads_.reserve(worker_count);
  try {
    for (std::size_t worker_index = 0; worker_index < worker_count;
         ++worker_index) {
      worker_threads_.emplace_back([this] { WorkerLoop(); });
    }
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      shutdown_requested_ = true;
    }
    task_available_condition_.notify_all();
    for (std::thread& worker_thread : worker_threads_) {
      if (worker_thread.joinable()) {
        worker_thread.join();
      }
    }
    throw;
  }
}

ThreadPool::~ThreadPool() { Shutdown(); }

void ThreadPool::Schedule(std::function<void()> task) {
  if (!task) {
    throw std::invalid_argument("scheduled task must not be empty");
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (shutdown_requested_) {
      throw std::logic_error("cannot schedule work after shutdown");
    }
    pending_tasks_.push(std::move(task));
  }
  task_available_condition_.notify_one();
}

void ThreadPool::Shutdown() {
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    shutdown_requested_ = true;
  }
  task_available_condition_.notify_all();
  for (std::thread& worker_thread : worker_threads_) {
    if (worker_thread.joinable()) {
      worker_thread.join();
    }
  }
  worker_threads_.clear();
}

void ThreadPool::WorkerLoop() {
  for (;;) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> queue_lock(queue_mutex_);
      task_available_condition_.wait(queue_lock, [this] {
        return shutdown_requested_ || !pending_tasks_.empty();
      });
      if (shutdown_requested_ && pending_tasks_.empty()) {
        return;
      }
      task = std::move(pending_tasks_.front());
      pending_tasks_.pop();
    }
    try {
      task();
    } catch (...) {
      // Submit() propagates exceptions through its future. Schedule() callers
      // own exception handling; keep an unexpected exception from killing a
      // worker and stranding the remaining queue.
    }
  }
}

}  // namespace common
}  // namespace atlas
