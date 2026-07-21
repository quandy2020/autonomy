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
 * @brief Monotonic and wall-clock time types for sensor HAL.
 */

#ifndef AUTODRIVER_COMMON_TIME_HPP_
#define AUTODRIVER_COMMON_TIME_HPP_

#include <chrono>
#include <cstdint>

/**
 * @namespace autodriver
 * @brief Sensor hardware abstraction layer.
 */
namespace autodriver {

/** Nanosecond-resolution duration. */
using Duration = std::chrono::nanoseconds;

/** Device/host timestamps in the HAL (UTC wall clock). */
using Timestamp = std::chrono::time_point<std::chrono::system_clock, Duration>;

/** Steady clock for measuring intervals inside drivers. */
using SteadyTime = std::chrono::steady_clock::time_point;

/**
 * @brief Returns the current wall-clock timestamp.
 */
inline Timestamp Now()
{
  return std::chrono::time_point_cast<Duration>(
    std::chrono::system_clock::now());
}

/**
 * @brief Builds a timestamp from nanoseconds since Unix epoch.
 */
inline Timestamp FromNanoseconds(int64_t nanoseconds)
{
  return Timestamp(Duration(nanoseconds));
}

/**
 * @brief Converts a timestamp to nanoseconds since Unix epoch.
 */
inline int64_t ToNanoseconds(Timestamp time)
{
  return time.time_since_epoch().count();
}

}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_TIME_HPP_
