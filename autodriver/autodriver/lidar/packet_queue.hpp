/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Bounded packet queue for lidar receive/convert decoupling.
 */

#ifndef AUTODRIVER_LIDAR_PACKET_QUEUE_HPP_
#define AUTODRIVER_LIDAR_PACKET_QUEUE_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace autodriver {
namespace lidar {

/**
 * @class autodriver::lidar::PacketQueue
 * @brief Thread-safe bounded FIFO; Push drops the oldest item when full (lossy).
 * @tparam T Element type (e.g. PacketBuffer).
 *
 * Use between a UDP read thread and a convert thread when
 * `packets_per_scan` aggregation is insufficient under bursty load.
 */
template <typename T>
class PacketQueue {
public:
    /**
     * @brief Construct with a maximum number of queued elements.
     * @param capacity Minimum capacity is 1.
     */
    explicit PacketQueue(std::size_t capacity = 256)
        : capacity_(capacity == 0 ? 1 : capacity) {}

    /**
     * @brief Enqueue @p item; if full, drop the front and count a drop.
     * @param item Value to move into the queue.
     * @return Always true (lossy drop still accepts the new item).
     */
    bool Push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= capacity_) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(item));
        return true;
    }

    /**
     * @brief Pop the oldest element if any.
     * @return nullopt when empty.
     */
    std::optional<T> TryPop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    /**
     * @brief Current number of queued elements.
     */
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /**
     * @brief Cumulative number of oldest-item drops due to capacity.
     */
    std::size_t dropped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

    /**
     * @brief Remove all queued elements (does not reset dropped()).
     */
    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }

private:
    // Maximum queue length.
    std::size_t capacity_;
    // Guards queue_ and dropped_.
    mutable std::mutex mutex_;
    // FIFO storage.
    std::deque<T> queue_;
    // Number of times Push dropped an oldest element.
    std::size_t dropped_ = 0;
};

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_PACKET_QUEUE_HPP_
