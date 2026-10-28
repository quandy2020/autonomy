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
 * @file bounded_queue.hpp
 * @brief Thread-safe blocking FIFO queue (mutex + condition variable).
 */

#ifndef ARGUS_COMMON_BOUNDED_QUEUE_HPP_
#define ARGUS_COMMON_BOUNDED_QUEUE_HPP_

#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

namespace argus {
namespace common {

/**
 * @brief Thread-safe FIFO backed by `std::queue`, a mutex, and a condition
 *        variable.
 * Supports non-blocking @ref Dequeue, blocking @ref WaitDequeue, and
 * cooperative shutdown via @ref BreakAllWait. Capacity is unbounded; the name
 * reflects historical usage as a bounded hand-off queue between threads.
 * @tparam T Element type; must be MoveAssignable for dequeue paths.
 */
template <typename T>
class BoundedQueue {
 public:
  BoundedQueue() = default;
  BoundedQueue(const BoundedQueue&) = delete;
  BoundedQueue& operator=(const BoundedQueue&) = delete;

  /**
   * @brief Destructor; unblocks all waiters via @ref BreakAllWait.
   */
  ~BoundedQueue() { BreakAllWait(); }

  /**
   * @brief Push a copy of @p element at the tail and wake one waiter.
   * @param[in] element Value to enqueue (copied).
   */
  void Enqueue(const T& element) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.emplace(element);
    cv_.notify_one();
  }

  /**
   * @brief Push @p element by move at the tail and wake one waiter.
   * @param[in,out] element Value to enqueue (moved-from afterward).
   */
  void Enqueue(T&& element) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.emplace(std::move(element));
    cv_.notify_one();
  }

  /**
   * @brief Pop the front element if the queue is non-empty (non-blocking).
   * @param[out] element Receives the popped value on success. Must be
   *                     non-null; otherwise behavior is undefined.
   * @return `true` if an element was written to @p element; `false` if the
   *         queue was empty ( @p element unchanged ).
   */
  bool Dequeue(T* element) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    *element = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  /**
   * @brief Block until an element is available or @ref BreakAllWait is
   *        called.
   * @param[out] element Receives the popped value when the wait succeeds.
   *                     Must be non-null.
   * @return `true` if an element was dequeued into @p element; `false` if
   *         the wait was cancelled by @ref BreakAllWait without data.
   */
  bool WaitDequeue(T* element) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() { return break_all_wait_ || !queue_.empty(); });
    if (break_all_wait_) {
      return false;
    }
    *element = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  /**
   * @brief Current number of queued elements.
   * @return Element count under the internal mutex.
   */
  typename std::queue<T>::size_type Size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  /**
   * @brief Whether the queue currently holds no elements.
   * @return `true` if empty; `false` otherwise.
   */
  bool Empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  /**
   * @brief Set the break flag and wake every blocked @ref WaitDequeue.
   * Subsequent @ref WaitDequeue calls return `false` until @ref ResetBreak.
   */
  void BreakAllWait() {
    break_all_wait_ = true;
    cv_.notify_all();
  }

  /**
   * @brief Clear the break flag so @ref WaitDequeue can block again.
   */
  void ResetBreak() { break_all_wait_ = false; }

 private:
  volatile bool break_all_wait_ = false;
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  std::condition_variable cv_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_BOUNDED_QUEUE_HPP_
