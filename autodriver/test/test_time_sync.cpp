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

#include "autodriver/common/time.hpp"
#include "autodriver/sync/time_sync.hpp"

using autodriver::FromNanoseconds;
using autodriver::Now;
using autodriver::TimeSync;
using autodriver::ToNanoseconds;

TEST(TimeSync, LearnsConstantOffset)
{
  TimeSync sync;
  const auto device = FromNanoseconds(1'000'000'000);
  const auto host = FromNanoseconds(1'500'000'000);

  for (int i = 0; i < 20; ++i) {
    sync.Update("imu/test", device, host);
  }

  EXPECT_NEAR(sync.GetOffsetNanoseconds("imu/test"), 500'000'000, 1'000'000);
  EXPECT_EQ(
    ToNanoseconds(sync.ToHostTime("imu/test", device)),
    ToNanoseconds(host));
}

TEST(TimeSync, UnknownSensorPassesThroughDeviceTime)
{
  TimeSync sync;
  const auto device = Now();
  EXPECT_EQ(
    ToNanoseconds(sync.ToHostTime("unknown", device)),
    ToNanoseconds(device));
}
