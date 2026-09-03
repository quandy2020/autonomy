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
 * @brief Intel RealSense RGB/depth/IR camera driver (D435i, D455, ...).
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
 * @class autodriver::hardware::RealSenseCameraDriver
 * @brief Publishes CameraFrame samples from a RealSense video stream.
 * Params:
 * - model (D455), serial, index
 * - stream (color, depth, ir1, ir2, aligned_depth_to_color)
 * - width, height, fps
 * - frame_id (optical frame override)
 * - emitter_enabled / enable_ir_emitter (device-wide, via shared hub)
 */
class RealSenseCameraDriver : public SensorDriver
{
public:
  /**
   * @brief Parses stream and resolution params and stores sensor identity.
   */
  RealSenseCameraDriver(SensorId id, DriverParams params);

  /**
   * @brief Unsubscribes and stops the shared device hub on destruction.
   */
  ~RealSenseCameraDriver() override;

  /**
   * @brief Report sensor type
   * @return SensorType::kCamera
   */
  SensorType GetType() const override { return SensorType::kCamera; }

  /**
   * @brief Return this driver's sensor identifier
   * @return Sensor id assigned at construction
   */
  const SensorId & GetSensorId() const override { return id_; }

  /**
   * @brief Subscribes to a RealSense video stream via the shared hub.
   */
  bool Start() override;

  /**
   * @brief Unsubscribes from the hub and releases the shared device handle.
   */
  void Stop() override;

  /**
   * @brief Returns true while the video subscription is active.
   */
  bool IsRunning() const override;

  /**
   * @brief Registers the callback invoked for each camera frame sample.
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  // Sensor identifier for this driver instance.
  SensorId id_;

  // Parsed driver parameters from configuration.
  DriverParams params_;

  // Selected RealSense video stream kind.
  realsense::StreamKind stream_{realsense::StreamKind::kColor};

  // Requested frame width in pixels.
  int width_{640};

  // Requested frame height in pixels.
  int height_{480};

  // Requested frames per second.
  int fps_{30};

  // Shared device hub managing the librealsense pipeline.
  std::shared_ptr<io::RealSenseDeviceHub> hub_;

  // Hub subscription handle returned by SubscribeVideo().
  std::uint64_t subscription_id_{0};

  // User callback for delivered camera samples.
  SampleCallback callback_;

  // True while Start() succeeded and Stop() has not been called.
  std::atomic<bool> running_{false};
};

/**
 * @brief Factory used by CameraModule.
 */
std::shared_ptr<SensorDriver> CreateRealSenseCameraDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_REALSENSE_CAMERA_DRIVER_HPP_
