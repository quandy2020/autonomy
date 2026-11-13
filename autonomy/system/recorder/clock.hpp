/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file clock.hpp
 * @brief Dual clock: CLOCK_MONOTONIC + CLOCK_REALTIME into DualClock.
 */

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Sample CLOCK_MONOTONIC and CLOCK_REALTIME into DualClock.
 * @param [in] source: Time-source tag (software / PTP / …).
 * @return DualClock with monotonic_ns, wall_utc_ns, and time_source.
 */
proto::DualClock NowDualClock(proto::TimeSource source =
                                  proto::TIME_SOURCE_SOFTWARE);

/**
 * @brief Build DualClock from a known monotonic stamp; one CLOCK_REALTIME read.
 *
 * Use when the middleware already supplies `msg->timestamp` so the hot path
 * skips a second CLOCK_MONOTONIC syscall.
 *
 * @param [in] monotonic_ns: Monotonic timestamp in nanoseconds.
 * @param [in] source: Time-source tag.
 * @return DualClock; wall_utc_ns comes from CLOCK_REALTIME.
 */
proto::DualClock DualClockFromMono(int64_t monotonic_ns,
                                   proto::TimeSource source =
                                       proto::TIME_SOURCE_SOFTWARE);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
