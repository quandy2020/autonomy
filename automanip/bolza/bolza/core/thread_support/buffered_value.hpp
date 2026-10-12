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

#pragma once

/**
 * @file buffered_value.hpp
 * @brief Wraps a value with a thread-safe buffer. A value remains constant and can be accessed through the get function until.
 */


#include <bolza/core/thread_support/synchronized.hpp>

namespace bolza {

/**
 * Wraps a value with a thread-safe buffer. A value remains constant and can be accessed through the get function until
 * updateFromBuffer is called.
 *
 * In the meantime, multiple threads can set new values to the buffer. The active value is not protected by a mutex, so
 * only one thread should access/modify the active Value (i.e. not simultaneously calling get() and UpdateFromBuffer()).
 *
 * @tparam T : wrapped type
 */
template <typename T>
class BufferedValue {
 public:
    /**
   * @brief /**.
   *
   * Constructor initializes with a given value and an empty buffer.
   *
   * /
   * @param [in] nullptr: Nullptr.
   */
  explicit BufferedValue(T value) : activeValue_(std::move(value)), buffer_(nullptr){};

  /**
   * @brief Read the currently active value.
   * @return Result value.
   */
  const T& get() const { return activeValue_; }

  /**
   * @brief Read/write the currently active value.
   * @return Result value.
   */
  T& get() { return activeValue_; }

    /**
   * @brief /** Copy a new value into the buffer. */.
   * @param [in] value: Value.
   */
  void SetBuffer(const T& value) { /**
  * @brief Reset.
  * @return Result value.
  */
 buffer_.reset(std::make_unique<T>(value)); }

    /**
   * @brief /** Move a new value into the buffer. */.
   * @param [in,out] value: Value.
   */
  void SetBuffer(T&& value) { /**
  * @brief Reset.
  * @return Result value.
  */
 buffer_.reset(std::make_unique<T>(std::move(value))); }

  /**
   * Replaces the active value with the value in the buffer.
   * The active value is not mutex protected so this method is NOT thread-safe w.r.t. get()
   * The buffer is mutex protected, so this method is thread-safe w.r.t. SetBuffer()
   * @return True: the active value was updated, False: the active value was not updated.
   */
  bool UpdateFromBuffer() {
    // Read buffer with a pointer swap to minimize time under the lock. The swapped value will be null if there was no new value set.
    /**
     * @brief Updated Value Ptr.
     * @param [in] nullptr: Nullptr.
     * @return Owning pointer.
     */
    std::unique_ptr<T> UpdatedValuePtr(nullptr);
    /**
     * @brief Swap.
     * @param [in] updatedValuePtr: Updated Value Ptr.
     * @return Result value.
     */
    buffer_.swap(updatedValuePtr);

    if (updatedValuePtr != nullptr) {
      /**
       * @brief Move.
       * @param [in,out] updatedValuePtr: Updated Value Ptr.
       * @return Result value.
       */
      activeValue_ = std::move(*updatedValuePtr);
      return true;
    } else {
      return false;
    }
  }

 private:
  T activeValue_;
  Synchronized<T> buffer_;
};

}  // namespace bolza
