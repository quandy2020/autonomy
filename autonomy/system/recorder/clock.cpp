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

/**
 * @file clock.cpp
 * @brief DualClock from CLOCK_MONOTONIC + CLOCK_REALTIME.
 */

#include "autonomy/system/recorder/clock.hpp"

#include <ctime>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

int64_t ClockNs(clockid_t id) {
    timespec ts{};
    clock_gettime(id, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
           static_cast<int64_t>(ts.tv_nsec);
}

}  // namespace

proto::DualClock NowDualClock(proto::TimeSource source) {
    return DualClockFromMono(ClockNs(CLOCK_MONOTONIC), source);
}

proto::DualClock DualClockFromMono(int64_t monotonic_ns,
                                   proto::TimeSource source) {
    proto::DualClock clock;
    clock.set_monotonic_ns(monotonic_ns);
    clock.set_wall_utc_ns(ClockNs(CLOCK_REALTIME));
    clock.set_time_source(source);
    return clock;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
