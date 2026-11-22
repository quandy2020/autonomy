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
 * @file time_util.cpp
 * @brief Dual clock via autolink::Time::{MonoTime,Now}.
 */

#include "common/base/time_util.hpp"

#include <chrono>
#include <thread>

#if defined(AUTOLOOP_WITH_AUTOLINK)
#include "autolink/time/duration.hpp"
#include "autolink/time/time.hpp"
#endif

namespace autoloop {
namespace common {

DualClock NowDualClock() {
    DualClock clock;
#if defined(AUTOLOOP_WITH_AUTOLINK)
    clock.set_mono_ns(
        static_cast<int64_t>(autolink::Time::MonoTime().ToNanosecond()));
    clock.set_utc_ns(
        static_cast<int64_t>(autolink::Time::Now().ToNanosecond()));
#else
    using clock_st = std::chrono::steady_clock;
    using clock_sys = std::chrono::system_clock;
    clock.set_mono_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                          clock_st::now().time_since_epoch())
                          .count());
    clock.set_utc_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                         clock_sys::now().time_since_epoch())
                         .count());
#endif
    return clock;
}

void SleepNs(int64_t ns) {
    if (ns <= 0) {
        return;
    }
#if defined(AUTOLOOP_WITH_AUTOLINK)
    autolink::Duration(ns).Sleep();
#else
    std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
#endif
}

}  // namespace common
}  // namespace autoloop
