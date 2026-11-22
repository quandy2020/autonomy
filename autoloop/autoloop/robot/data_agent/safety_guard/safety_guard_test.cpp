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

#include "robot/data_agent/safety_guard/safety_guard.hpp"
#include <gtest/gtest.h>
TEST(SafetyGuard, LatchHolds) {
    autoloop::robot::SafetyGuard g;
    g.Observe(true, false);
    EXPECT_TRUE(g.latched());
    g.Observe(false, false);
    EXPECT_TRUE(g.latched());
    g.Clear();
    EXPECT_FALSE(g.latched());
}
