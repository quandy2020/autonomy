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

#include "autolink/time/rate.hpp"

#include "autolink/common/log.hpp"

namespace autolink {

Rate::Rate(double frequency)
    : start_(Time::Now()),
      expected_cycle_time_(1.0 / frequency),
      actual_cycle_time_(0.0) {}

Rate::Rate(uint64_t nanoseconds)
    : start_(Time::Now()),
      expected_cycle_time_(static_cast<int64_t>(nanoseconds)),
      actual_cycle_time_(0.0) {}

Rate::Rate(const Duration& d)
    : start_(Time::Now()), expected_cycle_time_(d), actual_cycle_time_(0.0) {}

void Rate::Sleep() {
    Time actual_end = Time::Now();
    Time expected_end = start_ + expected_cycle_time_;

    // set the actual amount of time the loop took in case the user wants to know
    actual_cycle_time_ = actual_end - start_;

    // detect backward jumps in time (e.g., system clock adjustment, NTP sync)
    // Only warn if the jump is significant (> 100ms) to avoid false positives
    // from clock precision issues
    const int64_t BACKWARD_JUMP_THRESHOLD_NS = 100000000;  // 100ms
    if (actual_end < start_) {
        int64_t jump_ns = static_cast<int64_t>(start_.ToNanosecond()) - 
                          static_cast<int64_t>(actual_end.ToNanosecond());
        // Only warn for significant backward jumps (likely real clock adjustments)
        if (jump_ns > BACKWARD_JUMP_THRESHOLD_NS) {
            AWARN << "Detect backward jumps in time: start=" << start_.ToNanosecond()
                  << " ns, actual=" << actual_end.ToNanosecond() << " ns, jump="
                  << jump_ns << " ns";
        }
        // Reset start time to current time to handle the backward jump gracefully
        start_ = actual_end;
        expected_end = actual_end + expected_cycle_time_;
        // Set actual_cycle_time_ to zero since we can't calculate a valid cycle time
        actual_cycle_time_ = Duration(0.0);
    }

    // calculate the time we'll sleep for
    Duration sleep_time = expected_end - actual_end;

    // if we've taken too much time we won't sleep
    if (sleep_time < Duration(0.0)) {
        AWARN << "Detect forward jumps in time or loop took too long: "
              << "expected_end=" << expected_end.ToNanosecond()
              << " ns, actual_end=" << actual_end.ToNanosecond() << " ns";
        // if we've jumped forward in time, or the loop has taken more than a
        // full extra cycle, reset our cycle
        if (actual_end > expected_end + expected_cycle_time_) {
            start_ = actual_end;
        } else {
            // Update start_ to expected_end for next cycle to maintain rate
            start_ = expected_end;
        }
        // return false to show that the desired rate was not met
        return;
    }

    // Sleep until expected_end
    Time::SleepUntil(expected_end);
    
    // Update start_ to expected_end for next cycle
    // This ensures consistent rate: next cycle's expected_end will be start_ + cycle_time
    start_ = expected_end;
}

void Rate::Reset() {
    start_ = Time::Now();
}

Duration Rate::CycleTime() const {
    return actual_cycle_time_;
}

}  // namespace autolink
