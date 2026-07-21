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

#include "autodriver/app/sensor_manager.hpp"
#include "autodriver/config/hub_config.hpp"
#include "autodriver/sync/aligned_snapshot.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

TEST(SensorManager, LoadsDefaultSimulationStack)
{
  autodriver::SensorManager manager(autodriver::DefaultSimulationHubConfig());
  ASSERT_TRUE(manager.Initialize());
  EXPECT_TRUE(manager.init_errors().empty());
}

TEST(SensorManager, StartsAndPublishesAlignedFrames)
{
  autodriver::HubConfig config = autodriver::DefaultSimulationHubConfig();
  config.drivers = {{"mock_imu", "imu/test"}};
  config.hub_options.publish_period = std::chrono::milliseconds(10);

  autodriver::SensorManager manager(config);
  ASSERT_TRUE(manager.Initialize());

  std::atomic<int> frames{0};
  manager.SetAlignedCallback([&](const autodriver::AlignedSnapshot & snapshot) {
    ++frames;
    EXPECT_NE(snapshot.Get<autodriver::ImuSample>(autodriver::SensorType::kImu), nullptr);
  });

  ASSERT_TRUE(manager.Start());
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  manager.Stop();
  EXPECT_GT(frames.load(), 0);
}

TEST(SensorManager, ReportsUnknownDriver)
{
  autodriver::HubConfig config;
  config.register_builtin_mocks = false;
  config.drivers = {{"unknown_driver", "x"}};

  autodriver::SensorManager manager(config);
  EXPECT_FALSE(manager.Initialize());
  ASSERT_EQ(manager.init_errors().size(), 1u);
  EXPECT_EQ(manager.init_errors().front(), "unknown_driver");
}

TEST(SensorManager, RejectsMockDriverWhenMocksDisabled)
{
  autodriver::HubConfig config;
  config.register_builtin_mocks = false;
  config.drivers = {{"mock_imu", "imu/test"}};

  autodriver::SensorManager manager(config);
  EXPECT_FALSE(manager.Initialize());
  ASSERT_EQ(manager.init_errors().size(), 1u);
  EXPECT_EQ(manager.init_errors().front(), "mock_imu");
}
