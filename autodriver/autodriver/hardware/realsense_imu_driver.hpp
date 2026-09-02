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
 * @brief Intel RealSense on-board IMU driver (D435i, D455, ...).
 */

#ifndef AUTODRIVER_HARDWARE_REALSENSE_IMU_DRIVER_HPP_
#define AUTODRIVER_HARDWARE_REALSENSE_IMU_DRIVER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/hardware/realsense_device_hub.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::RealSenseImuDriver
 * @brief Reads fused accel/gyro motion streams from a RealSense D400 device.
 * Params: model, serial, index, frame_id.
 */
class RealSenseImuDriver : public SensorDriver
{
public:
  /**
   * @brief Stores sensor identity and driver params for RealSense IMU.
   */
  RealSenseImuDriver(SensorId id, DriverParams params);

  /**
   * @brief Unsubscribes and stops the shared device hub on destruction.
   */
  ~RealSenseImuDriver() override;

  /**
   * @brief Report sensor type
   * @return SensorType::kImu
   */
  SensorType GetType() const override { return SensorType::kImu; }

  /**
   * @brief Return this driver's sensor identifier
   * @return Sensor id assigned at construction
   */
  const SensorId & GetSensorId() const override { return id_; }

  /**
   * @brief Subscribes to fused accel/gyro frames via the shared hub.
   */
  bool Start() override;

  /**
   * @brief Unsubscribes from the hub and releases the shared device handle.
   */
  void Stop() override;

  /**
   * @brief Returns true while the IMU subscription is active.
   */
  bool IsRunning() const override;

  /**
   * @brief Registers the callback invoked for each IMU sample.
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  // Sensor identifier for this driver instance.
  SensorId id_;

  // Parsed driver parameters from configuration.
  DriverParams params_;

  // Shared device hub managing the librealsense pipeline.
  std::shared_ptr<io::RealSenseDeviceHub> hub_;

  // Hub subscription handle returned by SubscribeImu().
  std::uint64_t subscription_id_{0};

  // User callback for delivered IMU samples.
  SampleCallback callback_;

  // True while Start() succeeded and Stop() has not been called.
  std::atomic<bool> running_{false};
};

std::shared_ptr<SensorDriver> CreateRealSenseImuDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_REALSENSE_IMU_DRIVER_HPP_
