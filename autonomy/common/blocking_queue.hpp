/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>

#include "autonomy/common/port.hpp"
#include "autonomy/common/time.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
class BlockingQueue
{
public:
    static constexpr size_t kInfiniteQueueSize = 0;

    BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;

    explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

    void Push(T t) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return QueueNotFullCondition(); });
        deque_.push_back(std::move(t));
        cv_.notify_one();
    }

    bool PushWithTimeout(T t, const common::Duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, timeout,
                          [this]() { return QueueNotFullCondition(); })) {
            return false;
        }
        deque_.push_back(std::move(t));
        cv_.notify_one();
        return true;
    }

    T Pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return !QueueEmptyCondition(); });

        T t = std::move(deque_.front());
        deque_.pop_front();
        cv_.notify_one();
        return t;
    }

    T PopWithTimeout(const common::Duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, timeout,
                          [this]() { return !QueueEmptyCondition(); })) {
            return nullptr;
        }
        T t = std::move(deque_.front());
        deque_.pop_front();
        cv_.notify_one();
        return t;
    }

    template <typename R>
    R* PeekWithTimeout(const common::Duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, timeout,
                          [this]() { return !QueueEmptyCondition(); })) {
            return nullptr;
        }
        return deque_.front().get();
    }

    template <typename R>
    const R* Peek() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (deque_.empty()) {
            return nullptr;
        }
        return deque_.front().get();
    }

    size_t Size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return deque_.size();
    }

    void WaitUntilEmpty() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return QueueEmptyCondition(); });
    }

private:
    bool QueueEmptyCondition() const { return deque_.empty(); }

    bool QueueNotFullCondition() const {
        return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    const size_t queue_size_;
    std::deque<T> deque_;
};

}  // namespace common
}  // namespace autonomy
