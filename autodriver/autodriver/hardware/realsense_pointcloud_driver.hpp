/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Intel RealSense depth-to-color point cloud driver.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "autodriver/driver_params.hpp"
#include "autodriver/hardware/realsense_device_hub.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Factory used by Lidar3dModule.
 */
std::shared_ptr<SensorDriver> CreateRealSensePointCloudDriver(
    const SensorId& id, const DriverParams& params);

/**
 * @class autodriver::hardware::RealSensePointCloudDriver
 * @brief Publishes LidarCloud samples from a RealSense colored point cloud.
 */
class RealSensePointCloudDriver : public SensorDriver {
 public:
  /**
   * @brief Parses resolution params and stores sensor identity.
   */
  RealSensePointCloudDriver(SensorId id, DriverParams params);

  /**
   * @brief Unsubscribes and stops the shared device hub on destruction.
   */
  ~RealSensePointCloudDriver() override;

  /**
   * @brief Report sensor type
   * @return SensorType::kLidar3d
   */
  SensorType GetType() const override { return SensorType::kLidar3d; }

  /**
   * @brief Return this driver's sensor identifier
   * @return Sensor id assigned at construction
   */
  const SensorId& GetSensorId() const override { return id_; }

  /**
   * @brief Subscribes to a RealSense depth point cloud via the shared hub.
   */
  bool Start() override;

  /**
   * @brief Unsubscribes from the hub and releases the shared device handle.
   */
  void Stop() override;

  /**
   * @brief Returns true while the point-cloud subscription is active.
   */
  bool IsRunning() const override;

  /**
   * @brief Registers the callback invoked for each point-cloud sample.
   */
  void SetSampleCallback(SampleCallback callback) override;

 private:
    // Sensor identifier for this driver instance.
    SensorId id_;

    // Parsed driver parameters from configuration.
    DriverParams params_;

    // Requested depth map width in pixels.
    int width_{640};

    // Requested depth map height in pixels.
    int height_{480};

    // Requested frames per second.
    int fps_{30};

    // True while Start() succeeded and Stop() has not been called.
    std::atomic<bool> running_{false};

    // Shared device hub managing the librealsense pipeline.
    std::shared_ptr<io::RealSenseDeviceHub> hub_;

    // Hub subscription handle returned by SubscribePointCloud().
    std::uint64_t subscription_id_{0};

    // User callback for delivered point cloud samples.
    SampleCallback callback_;
};

}  // namespace hardware
}  // namespace autodriver
