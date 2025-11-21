/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <mutex>

#include "autolink/proto/run_mode_conf.pb.h"

#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/time/time.hpp"

namespace autolink {

using ::autolink::proto::ClockMode;

/**
 * @class Clock
 * @brief a singleton clock that can be used to get the current
 * timestamp. The source can be either system(autolink) clock or a mock clock.
 * Mock clock is for testing purpose mainly.
 */
class Clock
{
public:
    static constexpr int64_t PRECISION =
        std::chrono::system_clock::duration::period::den /
        std::chrono::system_clock::duration::period::num;

    /// PRECISION >= 1000000 means the precision is at least 1us.
    static_assert(PRECISION >= 1000000,
                  "The precision of the system clock should be at least 1 "
                  "microsecond.");

    /**
     * @brief get current time.
     * @return a Time object representing the current time.
     */
    static Time Now();

    /**
     * @brief gets the current time in second.
     * @return the current time in second.
     */
    static double NowInSeconds();

    /**
     * @brief This is for mock clock mode only. It will set the timestamp
     * for the mock clock.
     */
    static void SetNow(const Time& now);

    /**
     * @brief Set the behavior of the \class Clock.
     * @param The new clock mode to be set.
     */
    static void SetMode(ClockMode mode);

    /**
     * @brief Gets the current clock mode.
     * @return The current clock mode.
     */
    static ClockMode mode();

    /**
     * @brief This is for mock clock mode only. It will set the timestamp
     * for the mock clock with UNIX timestamp in seconds.
     */
    static void SetNowInSeconds(const double seconds) {
        Clock::SetNow(Time(seconds));
    }

private:
    ClockMode mode_;
    Time mock_now_;
    ::autolink::base::AtomicRWLock rwlock_;

    DECLARE_SINGLETON(Clock)
};

}  // namespace autolink