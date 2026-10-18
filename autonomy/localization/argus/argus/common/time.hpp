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
 * @brief Wall-clock helpers (seconds) for Argus / LIVO pipelines.
 */

#ifndef ARGUS_COMMON_TIME_HPP_
#define ARGUS_COMMON_TIME_HPP_

#include <chrono>
#include <cstdint>

namespace argus {
namespace common {

/** @return Unix epoch time in seconds (double). */
inline double NowSec() {
    using clock = std::chrono::system_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch())
        .count();
}

/** @return Monotonic steady-clock time in seconds (profiling / durations). */
inline double SteadyNowSec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch())
        .count();
}

/**
 * @brief Split floating seconds into integer sec + nanosec (ROS-style).
 * @param timestamp_sec Time since epoch in seconds.
 * @param sec Output whole seconds (ignored if null).
 * @param nanosec Output fractional part in nanoseconds (ignored if null).
 */
inline void SecToStamp(double timestamp_sec, int32_t* sec, uint32_t* nanosec) {
    if (sec == nullptr || nanosec == nullptr) {
        return;
    }
    auto s = static_cast<int32_t>(timestamp_sec);
    auto ns = static_cast<uint32_t>(
        (timestamp_sec - static_cast<double>(s)) * 1e9 + 0.5);
    if (ns >= 1000000000u) {
        *sec = s + 1;
        *nanosec = ns - 1000000000u;
    } else {
        *sec = s;
        *nanosec = ns;
    }
}

/**
 * @brief Combine integer sec and nanosec to floating seconds.
 * @param sec Whole seconds.
 * @param nanosec Fractional nanoseconds.
 * @return Time in seconds.
 */
inline double StampToSec(int32_t sec, uint32_t nanosec) {
    return static_cast<double>(sec) + 1e-9 * static_cast<double>(nanosec);
}

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_TIME_HPP_
