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

#include "robot/data_agent/ring_buffer/ring_buffer.hpp"

#include <gtest/gtest.h>

TEST(RingBuffer, PushTake) {
    autoloop::robot::RingBuffer buf(2'000'000'000);
    autoloop::robot::Sample s;
    s.channel = "/odom";
    s.clock.set_mono_ns(1);
    buf.Push(std::move(s));
    EXPECT_EQ(buf.size(), 1u);
    auto all = buf.TakeAll();
    EXPECT_EQ(all.size(), 1u);
    EXPECT_EQ(buf.size(), 0u);
}
