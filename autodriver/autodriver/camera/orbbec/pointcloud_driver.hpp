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
#include "autolink/common/macros.hpp"

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
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(OrbbecPointCloudDriver)

    /**
     * @brief Parse resolution / fps params (cold path).
     * @param id Sensor instance id.
     * @param params DriverParams from YAML.
     */
    OrbbecPointCloudDriver(SensorId id, DriverParams params);

    /**
     * @brief Unsubscribe and release the shared hub reference.
     */
    ~OrbbecPointCloudDriver() override;

    /**
     * @brief Report sensor type (depth cloud uses lidar3d sample path).
     * @return SensorType::kLidar3d.
     */
    SensorType GetSensorType() const override { return SensorType::kLidar3d; }

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    const SensorId& GetSensorId() const override { return id_; }

    /**
     * @brief Acquire OrbbecDeviceHub and subscribe to point cloud frames.
     * @return true on successful subscription / hub start.
     */
    bool Start() override;

    /**
     * @brief Unsubscribe and clear the running flag.
     */
    void Stop() override;

    /**
     * @brief Whether the driver subscription is active.
     * @return true after Start until Stop.
     */
    bool IsRunning() const override;

    /**
     * @brief Register the sample sink callback (hub callback thread).
     * @param callback May be empty to disable emission.
     */
    void SetSampleCallback(SampleCallback callback) override;

private:
    SensorId id_;
    DriverParams params_;
    int width_{640};
    int height_{480};
    int fps_{30};
    std::atomic<bool> running_{false};
    io::OrbbecDeviceHub::SharedPtr hub_{nullptr};
    std::uint64_t subscription_id_{0};
    SampleCallback callback_;
};

/**
 * @brief Factory for OrbbecPointCloudDriver (PointCloudBackendRegistry).
 * @param id Sensor instance id from YAML.
 * @param params Backend-specific key/value map.
 * @return Owning OrbbecPointCloudDriver*, or nullptr without OrbbecSDK.
 */
SensorDriver* CreateOrbbecPointCloudDriver(const SensorId& id,
                                           const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_CAMERA_ORBBEC_POINTCLOUD_DRIVER_HPP_
