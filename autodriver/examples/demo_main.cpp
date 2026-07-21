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

/**
 * @file
 * @brief Minimal demo: mock sensors through SensorHub.
 */

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "autodriver/drivers/mock/mock_drivers.hpp"
#include "autodriver/hal/sensor_factory.hpp"
#include "autodriver/sync/sensor_hub.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/lidar_scan.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autodriver/types/wheel_odometry_sample.hpp"

int main()
{
  autodriver::mock::RegisterMockDrivers();

  autodriver::SensorHub hub;
  hub.RegisterDriver(autodriver::mock::CreateMockImuDriver());
  hub.RegisterDriver(autodriver::mock::CreateMockWheelOdometryDriver());
  hub.RegisterDriver(autodriver::mock::CreateMockLidarDriver());

  std::atomic<int> frames{0};
  hub.SetAlignedCallback([&](const autodriver::AlignedSnapshot & snapshot) {
    const int n = ++frames;
    const auto * imu = snapshot.Get<autodriver::ImuSample>(autodriver::SensorType::kImu);
    const auto * odom = snapshot.Get<autodriver::WheelOdometrySample>(
      autodriver::SensorType::kWheelOdometry);
    const auto * lidar = snapshot.Get<autodriver::LidarScan>(
      autodriver::SensorType::kLidar);

    std::cout << "[frame " << n << "] sensors="
              << snapshot.samples.size()
              << " imu=" << (imu ? "yes" : "no")
              << " odom=" << (odom ? "yes" : "no")
              << " lidar=" << (lidar ? "yes" : "no")
              << std::endl;
  });

  if (!hub.Start()) {
    std::cerr << "SensorHub failed to start\n";
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));
  hub.Stop();
  std::cout << "Published " << frames.load() << " aligned frames\n";
  return 0;
}
