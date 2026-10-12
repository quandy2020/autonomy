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
 * @file benchmark.hpp
 * @brief Timer class that can be repeatedly started and stopped. Statistics are collected for all measured intervals .
 */


#include <chrono>

#include "bolza/core/types.hpp"

namespace bolza {
namespace benchmark {

/**
 * Timer class that can be repeatedly started and stopped. Statistics are collected for all measured intervals .
 */
class RepeatedTimer {
 public:
  /**
   * @brief Constructs a RepeatedTimer.
   */
  RepeatedTimer()
      /**
       * @brief Num Timed Intervals.
       * @return Result value.
       */
      : numTimedIntervals_(0),
        totalTime_(std::chrono::nanoseconds::zero()),
        maxIntervalTime_(std::chrono::nanoseconds::zero()),
        lastIntervalTime_(std::chrono::nanoseconds::zero()),
        startTime_(std::chrono::steady_clock::now()) {}

  /**
   *  Reset the timer statistics
   */
  void Reset() {
    numTimedIntervals_ = 0;
    /**
     * @brief Zero.
     * @return Result value.
     */
    totalTime_ = std::chrono::nanoseconds::zero();
    /**
     * @brief Zero.
     * @return Result value.
     */
    maxIntervalTime_ = std::chrono::nanoseconds::zero();
    /**
     * @brief Zero.
     * @return Result value.
     */
    lastIntervalTime_ = std::chrono::nanoseconds::zero();
  }

  /**
   *  Start timing an interval
   */
  void StartTimer() { /**
  * @brief Now.
  * @return Result value.
  */
 startTime_ = std::chrono::steady_clock::now(); }

  /**
   * Stop timing of an interval
   */
  void EndTimer() {
    /**
     * @brief Now.
     * @return Result value.
     */
    auto endTime = std::chrono::steady_clock::now();
    lastIntervalTime_ = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime_);
    /**
     * @brief Max.
     * @param [in] maxIntervalTime_: Max Interval Time.
     * @param [in] lastIntervalTime_: Last Interval Time.
     * @return Result value.
     */
    maxIntervalTime_ = std::max(maxIntervalTime_, lastIntervalTime_);
    totalTime_ += lastIntervalTime_;
    numTimedIntervals_++;
  };

  /**
   * @brief Get Num Timed Intervals.
   * @return Number of intervals that were timed
   */
  int GetNumTimedIntervals() const { return numTimedIntervals_; }

  /**
   * @brief Get Total In Milliseconds.
   * @return Total cumulative time of timed intervals
   */
  scalar_t GetTotalInMilliseconds() const { /**
  * @brief Count.
  * @return Result value.
  */
 return std::chrono::duration<scalar_t, std::milli>(totalTime_).count(); }

  /**
   * @brief Get Max Interval In Milliseconds.
   * @return Maximum duration of a single interval
   */
  scalar_t GetMaxIntervalInMilliseconds() const { /**
  * @brief Count.
  * @return Result value.
  */
 return std::chrono::duration<scalar_t, std::milli>(maxIntervalTime_).count(); }

  /**
   * @brief Get Last Interval In Milliseconds.
   * @return Duration of the last timed interval
   */
  scalar_t GetLastIntervalInMilliseconds() const { /**
  * @brief Count.
  * @return Result value.
  */
 return std::chrono::duration<scalar_t, std::milli>(lastIntervalTime_).count(); }

  /**
   * @brief Get Average In Milliseconds.
   * @return Average duration of all timed intervals
   */
  scalar_t GetAverageInMilliseconds() const { return GetTotalInMilliseconds() / numTimedIntervals_; }

 private:
  int numTimedIntervals_;
  std::chrono::nanoseconds totalTime_;
  std::chrono::nanoseconds maxIntervalTime_;
  std::chrono::nanoseconds lastIntervalTime_;
  std::chrono::steady_clock::time_point startTime_;
};

}  // namespace benchmark
}  // namespace bolza
