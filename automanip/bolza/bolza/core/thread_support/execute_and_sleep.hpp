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
 * @file execute_and_sleep.hpp
 * @brief Helper function to execute a given function and sleep for the remainder of the specified timing.
 */


#include <chrono>
#include <thread>

namespace bolza {

/**
 * Helper function to execute a given function and sleep for the remainder of the specified timing.
 * Will not interrupt the function if it is too slow.
 *
 * @tparam Functor : function type
 * @param [in] f: callable object to execute
 * @param [in] frequency: the frequency the function should run at.
 */
template <typename Functor>
/**
 * @brief Execute And Sleep.
 * @param [in] f: F.
 * @param [in] frequency: Frequency.
 */
void ExecuteAndSleep(Functor f, double frequency) {
  using clock = std::chrono::high_resolution_clock;
  /**
   * @brief Now.
   * @return Result value.
   */
  const auto start = clock::now();

  // Execute wrapped function
  /**
   * @brief F.
   */
  f();

  // Compute desired duration rounded to clock decimation
  /**
   * @brief Desired Duration.
   * @param [in] frequency: Frequency.
   * @return Result value.
   */
  const std::chrono::duration<double> desiredDuration(1.0 / frequency);
  const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

  // Sleep
  const auto sleepTill = start + dt;
  /**
   * @brief Sleep until.
   * @param [in] sleepTill: Sleep Till.
   * @return Result value.
   */
  std::this_thread::sleep_until(sleepTill);
}

/**
 * Helper function to execute a given function at a given rate while a condition is true.
 * Will not interrupt the function if it is too slow.
 *
 * @tparam Functor : function type
 * @param [in] f: callable object to execute.
 * @param [in] condition: condition checked every loop, will exit when this function returns false.
 * @param [in] frequency: the frequency the function should run at.
 */
template <typename Functor1, typename Functor2>
/**
 * @brief Execute At Rate.
 * @param [in] f: F.
 * @param [in] condition: Condition.
 * @param [in] frequency: Frequency.
 */
void ExecuteAtRate(Functor1 f, Functor2 condition, double frequency) {
  using clock = std::chrono::high_resolution_clock;

  // Compute desired duration rounded to clock decimation
  /**
   * @brief Desired Duration.
   * @param [in] frequency: Frequency.
   * @return Result value.
   */
  const std::chrono::duration<double> desiredDuration(1.0 / frequency);
  const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

  // Initialize timing
  /**
   * @brief Now.
   * @return Result value.
   */
  const auto start = clock::now();
  auto sleepTill = start + dt;

  // Execution loop
  while (condition()) {
    // Execute wrapped function
    /**
     * @brief F.
     */
    f();

    // Sleep
    /**
     * @brief Sleep until.
     * @param [in] sleepTill: Sleep Till.
     * @return Result value.
     */
    std::this_thread::sleep_until(sleepTill);
    sleepTill += dt;
  }
}

}  // namespace bolza