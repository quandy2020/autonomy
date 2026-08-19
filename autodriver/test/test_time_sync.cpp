/*
 * Copyright 2026 Autodriver contributors
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

#include <gtest/gtest.h>

#include "autodriver/time_synchronization.hpp"
#include "autolink/time/time.hpp"

using autodriver::TimeSync;

TEST(TimeSync, LearnsConstantOffset)
{
  TimeSync sync;
  const autolink::Time device(uint64_t{1'000'000'000});
  const autolink::Time host(uint64_t{1'500'000'000});

  for (int i = 0; i < 20; ++i) {
    sync.Update("imu/test", device, host);
  }

  EXPECT_NEAR(sync.OffsetNs("imu/test"), 500'000'000, 1'000'000);
  EXPECT_EQ(sync.ToHostTime("imu/test", device), host);
}

TEST(TimeSync, UnknownSensorPassesThroughDeviceTime)
{
  TimeSync sync;
  const autolink::Time device = autolink::Time::Now();
  EXPECT_EQ(sync.ToHostTime("unknown", device), device);
}
