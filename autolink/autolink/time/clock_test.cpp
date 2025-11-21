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

#include "autolink/time/clock.hpp"

#include "gtest/gtest.h"

using autolink::proto::ClockMode;

namespace autolink {

TEST(Clock, MockTime) {
    Clock::SetMode(ClockMode::MODE_AUTOLINK);
    EXPECT_EQ(ClockMode::MODE_AUTOLINK, Clock::mode());

    Clock::SetMode(ClockMode::MODE_MOCK);
    EXPECT_EQ(ClockMode::MODE_MOCK, Clock::mode());

    EXPECT_EQ(0, Clock::Now().ToNanosecond());

    Clock::SetNow(Time(1));
    EXPECT_EQ(1, Clock::Now().ToNanosecond());

    Clock::SetNowInSeconds(123.456);
    EXPECT_DOUBLE_EQ(123.456, Clock::NowInSeconds());
}

}  // namespace autolink