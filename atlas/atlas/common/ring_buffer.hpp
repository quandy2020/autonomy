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
 * @file ring_buffer.hpp
 * @brief Fixed-capacity lock-free ring buffer (one producer, one consumer).
 */

#ifndef ATLAS_COMMON_RING_BUFFER_HPP_
#define ATLAS_COMMON_RING_BUFFER_HPP_

#include <atomic>
#include <cstddef>
#include <optional>
#include <vector>

namespace atlas {
namespace common {

/**
 * @brief Fixed-capacity lock-free FIFO ring buffer.
 * Correct only when exactly one thread calls @ref TryPush and exactly one
 * (possibly different) thread calls @ref TryPop. Not safe for multiple
 * concurrent producers or consumers.
 * @tparam T Element type; trivially copyable is recommended.
 */
template <typename T>
class RingBuffer {
 public:
  /**
   * @brief Allocate a ring with the given usable capacity.
   * Internally stores `capacity + 1` slots so full and empty states remain
   * distinguishable.
   * @param[in] capacity Maximum number of elements that can be held at once.
   *                     Must be greater than zero for useful operation.
   */
  explicit RingBuffer(std::size_t capacity)
      : buf_(capacity + 1), cap_(capacity + 1) {}

  /**
   * @brief Attempt to push a copy of @p value at the producer index.
   * @param[in] value Element to store.
   * @return `true` if stored; `false` if the buffer is full ( @p value not
   *         written ).
   */
  bool TryPush(const T& value) {
    const std::size_t t = tail_.load(std::memory_order_relaxed);
    const std::size_t n = (t + 1) % cap_;
    if (n == head_.load(std::memory_order_acquire)) {
      return false;
    }
    buf_[t] = value;
    tail_.store(n, std::memory_order_release);
    return true;
  }

  /**
   * @brief Attempt to pop the oldest element at the consumer index.
   * @return Engaged `std::optional` with the element on success; empty
   *         optional if the buffer is empty.
   */
  std::optional<T> TryPop() {
    const std::size_t h = head_.load(std::memory_order_relaxed);
    if (h == tail_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }
    T value = buf_[h];
    head_.store((h + 1) % cap_, std::memory_order_release);
    return value;
  }

 private:
  std::vector<T> buf_;
  std::size_t cap_ = 0;
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_RING_BUFFER_HPP_
