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
 * @brief DualClock from autolink::Time::MonoTime + Time::Now.
 */

#include "autonomy/system/recorder/clock.hpp"

#include "autolink/time/time.hpp"

namespace autonomy {
namespace system {
namespace recorder {

proto::DualClock NowDualClock(proto::TimeSource source) {
    return DualClockFromMono(
        static_cast<int64_t>(autolink::Time::MonoTime().ToNanosecond()),
        source);
}

proto::DualClock DualClockFromMono(int64_t monotonic_ns,
                                   proto::TimeSource source) {
    proto::DualClock clock;
    clock.set_monotonic_ns(monotonic_ns);
    clock.set_wall_utc_ns(
        static_cast<int64_t>(autolink::Time::Now().ToNanosecond()));
    clock.set_time_source(source);
    return clock;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
