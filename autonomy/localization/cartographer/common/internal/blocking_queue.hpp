/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>

#include "autonomy/localization/cartographer/common/port.hpp"
#include "autonomy/localization/cartographer/common/time.hpp"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
class BlockingQueue
{
public:
    static constexpr size_t kInfiniteQueueSize = 0;

    // Constructs a blocking queue with infinite queue size.
    BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;

    // Constructs a blocking queue with a size of 'queue_size'.
    explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

    // Pushes a value onto the queue. Blocks if the queue is full.
    void Push(T t) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return QueueNotFullCondition(); });
        deque_.push_back(std::move(t));
        condition_.notify_one();
    }

    // Like push, but returns false if 'timeout' is reached.
    bool PushWithTimeout(T t, const common::Duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!condition_.wait_for(lock, timeout, [this] { return QueueNotFullCondition(); })) {
            return false;
        }
        deque_.push_back(std::move(t));
        condition_.notify_one();
        return true;
    }

    // Pops the next value from the queue. Blocks until a value is available.
    T Pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return !QueueEmptyCondition(); });

        T t = std::move(deque_.front());
        deque_.pop_front();
        condition_.notify_one();
        return t;
    }

    // Like Pop, but can timeout. Returns nullptr in this case.
    T PopWithTimeout(const common::Duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!condition_.wait_for(lock, timeout, [this] { return !QueueEmptyCondition(); })) {
            return nullptr;
        }
        T t = std::move(deque_.front());
        deque_.pop_front();
        condition_.notify_one();
        return t;
    }

    // Like Peek, but can timeout. Returns nullptr in this case.
    template <typename R>
    R* PeekWithTimeout(const common::Duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!condition_.wait_for(lock, timeout, [this] { return !QueueEmptyCondition(); })) {
            return nullptr;
        }
        return deque_.front().get();
    }

    // Returns the next value in the queue or nullptr if the queue is empty.
    // Maintains ownership. This assumes a member function get() that returns
    // a pointer to the given type R.
    template <typename R>
    const R* Peek() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (deque_.empty()) {
            return nullptr;
        }
        return deque_.front().get();
    }

    // Returns the number of items currently in the queue.
    size_t Size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return deque_.size();
    }

    // Blocks until the queue is empty.
    void WaitUntilEmpty() {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return QueueEmptyCondition(); });
    }

private:
    // Returns true iff the queue is empty.
    bool QueueEmptyCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
        return deque_.empty();
    }

    // Returns true iff the queue is not full.
    bool QueueNotFullCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
        return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
    }

    std::mutex mutex_;
    std::condition_variable condition_;
    const size_t queue_size_ GUARDED_BY(mutex_);
    std::deque<T> deque_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
