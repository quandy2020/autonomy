/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "atlas/mapping/local_mapping_idle_monitor.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <future>

namespace autonomy::localization::atlas::internal {
namespace {

using namespace std::chrono_literals;

TEST(LocalMappingIdleMonitorTest, WakesOnlyWhenQueueEmptyAndAccepting) {
    LocalMappingIdleMonitor monitor;
    monitor.SetAccepting(false);
    monitor.SetQueueEmpty(false);

    auto result = std::async(
        std::launch::async, [&monitor]() { return monitor.WaitUntilIdle(1s); });

    monitor.SetQueueEmpty(true);
    EXPECT_EQ(result.wait_for(20ms), std::future_status::timeout);

    monitor.SetAccepting(true);
    ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
    EXPECT_EQ(result.get(), LocalMappingIdleWaitResult::kIdle);
}

TEST(LocalMappingIdleMonitorTest, TimesOutWhenMappingRemainsBusy) {
    LocalMappingIdleMonitor monitor;
    monitor.SetQueueEmpty(false);

    EXPECT_EQ(monitor.WaitUntilIdle(1ms),
              LocalMappingIdleWaitResult::kTimedOut);
}

TEST(LocalMappingIdleMonitorTest, InterruptWakesBlockedWaiter) {
    LocalMappingIdleMonitor monitor;
    monitor.SetQueueEmpty(false);

    auto result = std::async(
        std::launch::async, [&monitor]() { return monitor.WaitUntilIdle(1s); });
    monitor.Interrupt();

    ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
    EXPECT_EQ(result.get(), LocalMappingIdleWaitResult::kInterrupted);
}

TEST(LocalMappingIdleMonitorTest, InterruptAppliesToNewWaitersUntilResume) {
    LocalMappingIdleMonitor monitor;
    monitor.Interrupt();

    EXPECT_EQ(monitor.WaitUntilIdle(0ms),
              LocalMappingIdleWaitResult::kInterrupted);

    monitor.Resume();
    EXPECT_TRUE(monitor.IsIdle());
    EXPECT_EQ(monitor.WaitUntilIdle(0ms), LocalMappingIdleWaitResult::kIdle);
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
