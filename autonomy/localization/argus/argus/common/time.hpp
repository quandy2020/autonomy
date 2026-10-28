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
 * @file time.hpp
 * @brief Wall-clock and stamp conversion helpers (seconds) for Argus / LIVO.
 */

#ifndef ARGUS_COMMON_TIME_HPP_
#define ARGUS_COMMON_TIME_HPP_

#include <chrono>
#include <cstdint>

namespace argus {
namespace common {

/**
 * @brief Current Unix-epoch wall time from `std::chrono::system_clock`.
 * @return Seconds since epoch as a double (fractional part included).
 */
inline double NowSeconds() {
  using clock = std::chrono::system_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

/**
 * @brief Current monotonic time from `std::chrono::steady_clock`.
 * Prefer this for profiling and duration measurements; it is not affected by
 * wall-clock adjustments.
 * @return Seconds since an unspecified steady epoch (double).
 */
inline double SteadyNowSeconds() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

/**
 * @brief Split floating seconds into integer seconds and nanoseconds
 *        (ROS-style stamp).
 * @param[in]  timestamp_seconds Time since epoch in seconds.
 * @param[out] seconds           Whole seconds. Untouched if null.
 * @param[out] nanoseconds       Fractional part in `[0, 1e9)`. Untouched if
 *                               null. Both pointers must be non-null or the
 *                               call is a no-op.
 */
inline void SecondsToStamp(double timestamp_seconds, int32_t* seconds,
                           uint32_t* nanoseconds) {
  if (seconds == nullptr || nanoseconds == nullptr) {
    return;
  }
  auto whole_seconds = static_cast<int32_t>(timestamp_seconds);
  auto fractional_nanoseconds = static_cast<uint32_t>(
      (timestamp_seconds - static_cast<double>(whole_seconds)) * 1e9 + 0.5);
  if (fractional_nanoseconds >= 1000000000u) {
    *seconds = whole_seconds + 1;
    *nanoseconds = fractional_nanoseconds - 1000000000u;
  } else {
    *seconds = whole_seconds;
    *nanoseconds = fractional_nanoseconds;
  }
}

/**
 * @brief Combine integer seconds and nanoseconds into floating seconds.
 * @param[in] seconds     Whole seconds since epoch.
 * @param[in] nanoseconds Fractional nanoseconds in a typical ROS stamp range.
 * @return `seconds + 1e-9 * nanoseconds` as double.
 */
inline double StampToSeconds(int32_t seconds, uint32_t nanoseconds) {
  return static_cast<double>(seconds) + 1e-9 * static_cast<double>(nanoseconds);
}

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_TIME_HPP_
