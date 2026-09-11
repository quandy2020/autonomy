/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file pointcloud_driver.hpp
 * @brief Intel RealSense depth-to-color point cloud driver.
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "autodriver/driver_params.hpp"
#include "autodriver/camera/realsense/device_hub.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Factory used by Lidar3dModule.
 * @param[in] id Sensor instance id from YAML.
 * @param[in] params Backend-specific key/value map.
 * @return Owning RealSensePointCloudDriver* (never null).
 */
SensorDriver*
CreateRealSensePointCloudDriver(
    const SensorId& id, const DriverParams& params);

/**
 * @class autodriver::hardware::RealSensePointCloudDriver
 * @brief Publishes LidarCloud samples from a RealSense colored point cloud.
 */
class RealSensePointCloudDriver : public SensorDriver {
 public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(RealSensePointCloudDriver)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(RealSensePointCloudDriver)
  /**
   * @brief Parses resolution params and stores sensor identity.
   * @param[in] id Sensor instance id.
   * @param[in] params DriverParams (cold-path parse only).
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
  SensorType GetSensorType() const override { return SensorType::kLidar3d; }

  /**
   * @brief Return this driver's sensor identifier
   * @return Sensor id assigned at construction
   */
  const SensorId& GetSensorId() const override { return id_; }

  /**
   * @brief Subscribes to a RealSense depth point cloud via the shared hub.
   * @return True on successful subscription / hub start.
   */
  bool Start() override;

  /**
   * @brief Unsubscribes from the hub and releases the shared device handle.
   */
  void Stop() override;

  /**
   * @brief Returns true while the point-cloud subscription is active.
   * @return True after Start until Stop.
   */
  bool IsRunning() const override;

  /**
   * @brief Registers the callback invoked for each point-cloud sample.
   * @param[in] callback May be empty to disable emission.
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
    io::RealSenseDeviceHub::SharedPtr hub_{nullptr};

    // Hub subscription handle returned by SubscribePointCloud().
    std::uint64_t subscription_id_{0};

    // User callback for delivered point cloud samples.
    SampleCallback callback_;
};

}  // namespace hardware
}  // namespace autodriver
