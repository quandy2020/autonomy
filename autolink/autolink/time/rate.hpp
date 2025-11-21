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

#include "autolink/time/duration.hpp"
#include "autolink/time/time.hpp"

namespace autolink {

class Rate
{
public:
    explicit Rate(double frequency);
    explicit Rate(uint64_t nanoseconds);
    explicit Rate(const Duration&);
    void Sleep();
    void Reset();
    Duration CycleTime() const;
    Duration ExpectedCycleTime() const {
        return expected_cycle_time_;
    }

private:
    Time start_;
    Duration expected_cycle_time_;
    Duration actual_cycle_time_;
};

}  // namespace autolink