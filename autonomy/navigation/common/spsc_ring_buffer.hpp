/*
 * Copyright 2026 The Navigation Authors (navigation)
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

/**
 * @file spsc_ring_buffer.hpp
 * @brief Lock-free bounded single-producer single-consumer queue.
 */

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace navigation
{
namespace common
{

/**
 * @brief Bounded SPSC queue with power-of-two capacity.
 *
 * @tparam T Element type; must be default constructible and copy assignable.
 * @tparam Capacity Number of slots; must be a power of two.
 *
 * @par Threading
 * One producer calls TryPush; one consumer calls TryPop. Size() is advisory.
 */
template <typename T, std::size_t Capacity>
class SpscRingBuffer
{
    static_assert(Capacity >= 2, "capacity must be at least two");
    static_assert((Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");
    static_assert(std::is_default_constructible<T>::value, "T must be default constructible");

public:
    /** @brief Construct an empty queue. */
    SpscRingBuffer() = default;

    SpscRingBuffer(const SpscRingBuffer&) = delete;
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;

    /**
     * @brief Try to enqueue one element.
     *
     * @param[in] value Element copied into the queue.
     * @return false when the queue is full.
     */
    bool TryPush(const T& value)
    {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & (Capacity - 1);
        if (next == tail_.load(std::memory_order_acquire))
        {
            return false;
        }
        buffer_[head] = value;
        head_.store(next, std::memory_order_release);
        return true;
    }

    /**
     * @brief Try to dequeue one element.
     *
     * @param[out] value Destination for the oldest element.
     * @return false when the queue is empty.
     */
    bool TryPop(T* value)
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire))
        {
            return false;
        }
        *value = buffer_[tail];
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    /** @brief Approximate number of queued elements. */
    [[nodiscard]] std::size_t Size() const
    {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & (Capacity - 1);
    }

    /** @brief Number of usable slots (Capacity - 1). */
    [[nodiscard]] static constexpr std::size_t CapacityValue()
    {
        return Capacity - 1;
    }

private:
    /** @brief Producer index; cache-line isolated. */
    alignas(64) std::atomic<std::size_t> head_{0};
    /** @brief Consumer index; cache-line isolated. */
    alignas(64) std::atomic<std::size_t> tail_{0};
    /** @brief Element storage. */
    std::array<T, Capacity> buffer_{};
};

}  // namespace common
}  // namespace navigation
