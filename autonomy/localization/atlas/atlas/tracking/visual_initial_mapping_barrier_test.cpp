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

#include "atlas/tracking/visual_initial_mapping_barrier.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

namespace autonomy::localization::atlas::internal {
namespace {

TEST(VisualInitialMappingBarrierPolicyTest, EnablesMonocularAndImuRgbdOnly) {
    EXPECT_TRUE(ShouldUseVisualInitialMappingBarrier(true, false));
    EXPECT_TRUE(ShouldUseVisualInitialMappingBarrier(false, true));
    EXPECT_FALSE(ShouldUseVisualInitialMappingBarrier(false, false));
}

TEST(VisualInitialMappingBarrierTest, ConsumesPendingEventOnlyOnce) {
    VisualInitialMappingBarrier barrier;
    EXPECT_FALSE(barrier.Consume());

    barrier.MarkPending();
    EXPECT_TRUE(barrier.Consume());
    EXPECT_FALSE(barrier.Consume());
}

TEST(VisualInitialMappingBarrierTest, CanBeRearmedAfterConsumption) {
    VisualInitialMappingBarrier barrier;
    barrier.MarkPending();
    ASSERT_TRUE(barrier.Consume());

    barrier.MarkPending();
    EXPECT_TRUE(barrier.Consume());
    EXPECT_FALSE(barrier.Consume());
}

TEST(VisualInitialMappingBarrierTest, ConcurrentConsumersHaveOneWinner) {
    VisualInitialMappingBarrier barrier;
    barrier.MarkPending();
    std::atomic<int> successful_consumers{0};
    std::vector<std::thread> consumers;
    consumers.reserve(8);
    for (int index = 0; index < 8; ++index) {
        consumers.emplace_back([&barrier, &successful_consumers]() {
            if (barrier.Consume())
                successful_consumers.fetch_add(1, std::memory_order_relaxed);
        });
    }
    for (std::thread& consumer : consumers)
        consumer.join();

    EXPECT_EQ(successful_consumers.load(std::memory_order_relaxed), 1);
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
