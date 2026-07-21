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
 * @brief Mock sensor drivers for simulation and unit tests.
 */

#ifndef AUTODRIVER_DRIVERS_MOCK_MOCK_DRIVERS_HPP_
#define AUTODRIVER_DRIVERS_MOCK_MOCK_DRIVERS_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/hal/sensor_driver.hpp"
#include "autodriver/hal/sensor_factory.hpp"

namespace autodriver {
namespace mock {

/** @brief Registers all built-in mock drivers with SensorFactory. */
void RegisterMockDrivers();

/** @brief Creates a periodic mock IMU driver. */
std::shared_ptr<SensorDriver> CreateMockImuDriver(
  SensorId sensor_id = "imu/mock",
  std::chrono::milliseconds period = std::chrono::milliseconds(10));

/** @brief Creates a periodic mock GNSS driver. */
std::shared_ptr<SensorDriver> CreateMockGpsDriver(
  SensorId sensor_id = "gps/mock",
  std::chrono::milliseconds period = std::chrono::milliseconds(200));

/** @brief Creates a periodic mock wheel odometry driver. */
std::shared_ptr<SensorDriver> CreateMockWheelOdometryDriver(
  SensorId sensor_id = "wheel_odom/mock",
  std::chrono::milliseconds period = std::chrono::milliseconds(20));

/** @brief Creates a periodic mock 2-D LiDAR driver. */
std::shared_ptr<SensorDriver> CreateMockLidarDriver(
  SensorId sensor_id = "lidar/mock",
  std::chrono::milliseconds period = std::chrono::milliseconds(100));

/** @brief Creates a periodic mock range finder driver. */
std::shared_ptr<SensorDriver> CreateMockRangeFinderDriver(
  SensorId sensor_id = "range/mock",
  std::chrono::milliseconds period = std::chrono::milliseconds(50));

/** @brief Creates a periodic mock camera driver. */
std::shared_ptr<SensorDriver> CreateMockCameraDriver(
  SensorId sensor_id = "camera/mock",
  std::chrono::milliseconds period = std::chrono::milliseconds(33));

/** @brief Creates a driver by factory name and sensor instance id. */
std::shared_ptr<SensorDriver> CreateByName(
  const std::string & factory_name,
  const SensorId & sensor_id = {});

}  // namespace mock
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVERS_MOCK_MOCK_DRIVERS_HPP_
