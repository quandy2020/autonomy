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

#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "autodriver/drivers/mock/mock_drivers.hpp"
#include "autodriver/sync/aligned_snapshot.hpp"
#include "autodriver/sync/sensor_hub.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

using autodriver::SensorHub;
using autodriver::SensorHubOptions;
using autodriver::SensorType;
using autodriver::ImuSample;

TEST(SensorHub, PublishesAlignedSnapshotsWithMockImu)
{
  SensorHubOptions options;
  options.publish_period = std::chrono::milliseconds(10);
  options.alignment_window = std::chrono::milliseconds(200);

  SensorHub hub(options);
  hub.RegisterDriver(autodriver::mock::CreateMockImuDriver());

  std::atomic<int> aligned_count{0};
  hub.SetAlignedCallback([&](const autodriver::AlignedSnapshot & snapshot) {
    ++aligned_count;
    EXPECT_NE(snapshot.Get<ImuSample>(SensorType::kImu), nullptr);
  });

  ASSERT_TRUE(hub.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  hub.Stop();

  EXPECT_GT(aligned_count.load(), 0);
}
