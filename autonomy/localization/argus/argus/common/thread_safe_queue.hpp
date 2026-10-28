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

#ifndef ARGUS_COMMON_THREAD_SAFE_QUEUE_HPP_
#define ARGUS_COMMON_THREAD_SAFE_QUEUE_HPP_

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <utility>

namespace argus {
namespace common {

// Bounded queue. TryPush never blocks. A full queue returns false so the
// frontend cannot stall on the backend.
template <typename T>
class ThreadSafeQueue {
 public:
  explicit ThreadSafeQueue(std::size_t capacity) : capacity_(capacity) {
    if (capacity_ == 0) {
      throw std::invalid_argument("ThreadSafeQueue capacity must be > 0");
    }
  }

  ThreadSafeQueue(const ThreadSafeQueue&) = delete;
  ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

  bool TryPush(const T& value) { return Push(value); }
  bool TryPush(T&& value) { return Push(std::move(value)); }

  bool TryPop(T* value) {
    if (value == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    *value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  // Pops one item. Blocks while the queue is empty and still open. After
  // Close(), remaining items can still be popped; an empty closed queue
  // returns false.
  bool WaitPop(T* value) {
    if (value == nullptr) {
      return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) {
      return false;
    }
    *value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void Close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    cv_.notify_all();
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

 private:
  template <typename U>
  bool Push(U&& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_ || queue_.size() >= capacity_) {
      return false;
    }
    queue_.push(std::forward<U>(value));
    cv_.notify_one();
    return true;
  }

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<T> queue_;
  std::size_t capacity_ = 0;
  bool closed_ = false;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_THREAD_SAFE_QUEUE_HPP_
