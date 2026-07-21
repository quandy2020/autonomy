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
 * @brief Intel RealSense on-board IMU driver (D435i, ...).
 */

#ifndef AUTODRIVER_DRIVERS_HARDWARE_REALSENSE_IMU_DRIVER_HPP_
#define AUTODRIVER_DRIVERS_HARDWARE_REALSENSE_IMU_DRIVER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/drivers/hardware/driver_params.hpp"
#include "autodriver/hal/sensor_driver.hpp"
#include "autodriver/io/realsense_device.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class RealSenseImuDriver
 * @brief Reads accel/gyro motion streams from a RealSense D400 device.
 */
class RealSenseImuDriver : public SensorDriver
{
public:
  RealSenseImuDriver(SensorId sensor_id, DriverParams params);
  ~RealSenseImuDriver() override;

  SensorType GetType() const override { return SensorType::kImu; }
  const SensorId & GetSensorId() const override { return sensor_id_; }
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override;
  void SetSampleCallback(SampleCallback callback) override;

private:
  SensorId sensor_id_;
  DriverParams params_;
  std::shared_ptr<io::RealSenseDeviceHub> hub_;
  std::uint64_t subscription_id_{0};
  SampleCallback callback_;
  std::atomic<bool> running_{false};
};

std::shared_ptr<SensorDriver> CreateRealSenseImuDriver(
  const SensorId & sensor_id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVERS_HARDWARE_REALSENSE_IMU_DRIVER_HPP_
