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
 * @file thread_safe_queue.hpp
 * @brief Mutex + CV blocking queue (ported from autolink/base).
 */

#ifndef ARGUS_COMMON_THREAD_SAFE_QUEUE_HPP_
#define ARGUS_COMMON_THREAD_SAFE_QUEUE_HPP_

#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

namespace argus {
namespace common {

/**
 * @brief Thread-safe FIFO backed by `std::queue`, mutex, and condition
 * variable.
 *
 * Supports non-blocking dequeue, blocking wait-dequeue, and cooperative
 * shutdown via @ref BreakAllWait.
 */
template <typename T>
class ThreadSafeQueue
{
public:
    ThreadSafeQueue() = default;
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /** @brief Unblocks all waiters (same as @ref BreakAllWait). */
    ~ThreadSafeQueue() {
        BreakAllWait();
    }

    /** @brief Push a copy at the tail and notify one waiter. @param element
     * Value to enqueue. */
    void Enqueue(const T& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.emplace(element);
        cv_.notify_one();
    }

    /** @brief Push by move at the tail and notify one waiter. @param element
     * Value to enqueue. */
    void Enqueue(T&& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.emplace(std::move(element));
        cv_.notify_one();
    }

    /**
     * @brief Pop the front element if the queue is non-empty.
     * @param[out] element Receives the popped value on success.
     * @return False if the queue was empty.
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
     * called.
     * @param[out] element Receives the popped value when successful.
     * @return False if woken by break-all-wait without an element.
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

    /** @return Current number of queued elements. */
    typename std::queue<T>::size_type Size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /** @return True if the queue has no elements. */
    bool Empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /** @brief Set break flag and wake all blocked @ref WaitDequeue callers. */
    void BreakAllWait() {
        break_all_wait_ = true;
        cv_.notify_all();
    }

    /** @brief Clear the break flag so @ref WaitDequeue can block again. */
    void ResetBreak() {
        break_all_wait_ = false;
    }

private:
    volatile bool break_all_wait_ = false;
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cv_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_THREAD_SAFE_QUEUE_HPP_
