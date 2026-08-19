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
 * @brief Intel RealSense RGB/depth camera driver (D435i, D455, ...).
 */

#ifndef AUTODRIVER_HARDWARE_REALSENSE_CAMERA_DRIVER_HPP_
#define AUTODRIVER_HARDWARE_REALSENSE_CAMERA_DRIVER_HPP_

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
 * @class RealSenseCameraDriver
 * @brief Publishes CameraFrame samples from a RealSense video stream.
 */
class RealSenseCameraDriver : public SensorDriver
{
public:
  RealSenseCameraDriver(SensorId id, DriverParams params);
  ~RealSenseCameraDriver() override;

  SensorType GetType() const override { return SensorType::kCamera; }
  const SensorId & GetSensorId() const override { return id_; }
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override;
  void SetSampleCallback(SampleCallback callback) override;

private:
  SensorId id_;
  DriverParams params_;
  realsense::StreamKind stream_{realsense::StreamKind::kColor};
  int width_{640};
  int height_{480};
  int fps_{30};
  std::shared_ptr<io::RealSenseDeviceHub> hub_;
  std::uint64_t subscription_id_{0};
  SampleCallback callback_;
  std::atomic<bool> running_{false};
};

std::shared_ptr<SensorDriver> CreateRealSenseCameraDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_REALSENSE_CAMERA_DRIVER_HPP_
