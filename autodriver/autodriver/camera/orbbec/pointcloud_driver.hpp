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
 * @brief Orbbec depth/RGB point cloud driver.
 */

#ifndef AUTODRIVER_CAMERA_ORBBEC_POINTCLOUD_DRIVER_HPP_
#define AUTODRIVER_CAMERA_ORBBEC_POINTCLOUD_DRIVER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>

#include "autodriver/camera/orbbec/device_hub.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::OrbbecPointCloudDriver
 * @brief Publishes LidarCloud samples from an Orbbec depth/RGB point cloud.
 *
 * Params: serial, index, model, width, height, fps, frame_id.
 */
class OrbbecPointCloudDriver : public SensorDriver {
public:
    OrbbecPointCloudDriver(SensorId id, DriverParams params);
    ~OrbbecPointCloudDriver() override;

    SensorType GetType() const override { return SensorType::kLidar3d; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

private:
    SensorId id_;
    DriverParams params_;
    int width_{640};
    int height_{480};
    int fps_{30};
    std::atomic<bool> running_{false};
    std::shared_ptr<io::OrbbecDeviceHub> hub_;
    std::uint64_t subscription_id_{0};
    SampleCallback callback_;
};

/**
 * @brief Factory for OrbbecPointCloudDriver (PointCloudBackendRegistry).
 */
std::shared_ptr<SensorDriver> CreateOrbbecPointCloudDriver(
    const SensorId& id, const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_CAMERA_ORBBEC_POINTCLOUD_DRIVER_HPP_
