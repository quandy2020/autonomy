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
 * @file camera_driver.hpp
 * @brief Orbbec RGB/depth/IR camera driver.
 */

#ifndef AUTODRIVER_CAMERA_ORBBEC_CAMERA_DRIVER_HPP_
#define AUTODRIVER_CAMERA_ORBBEC_CAMERA_DRIVER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/camera/orbbec/device_hub.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::OrbbecCameraDriver
 * @brief Publishes CameraFrame from an Orbbec video stream via OrbbecDeviceHub.
 *
 * Params: serial, index, model, stream (color|depth|ir), width, height, fps,
 * frame_id.
 */
class OrbbecCameraDriver : public SensorDriver {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(OrbbecCameraDriver)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(OrbbecCameraDriver)
    /**
     * @brief Parse stream / resolution params (cold path).
     * @param[in] id Sensor instance id.
     * @param[in] params DriverParams from YAML.
     */
    OrbbecCameraDriver(SensorId id, DriverParams params);

    /**
     * @brief Unsubscribe and stop the shared hub if this was the last user.
     */
    ~OrbbecCameraDriver() override;

    /**
     * @brief Report sensor type.
     * @return SensorType::kCamera.
     */
    SensorType GetSensorType() const override { return SensorType::kCamera; }

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    const SensorId& GetSensorId() const override { return id_; }

    /**
     * @brief Acquire OrbbecDeviceHub and subscribe to the configured stream.
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
     * @param[in] callback May be empty to disable emission.
     */
    void SetSampleCallback(SampleCallback callback) override;

private:
    // Sensor identifier for this driver instance.
    SensorId id_;
    // Parsed driver parameters from configuration.
    DriverParams params_;
    // Color / depth / IR stream selection.
    orbbec::StreamKind stream_{orbbec::StreamKind::kColor};
    int width_{640};
    int height_{480};
    int fps_{30};
    // Shared pipeline hub for this physical device.
    io::OrbbecDeviceHub::SharedPtr hub_{nullptr};
    // Hub subscription token; 0 when unsubscribed.
    std::uint64_t subscription_id_{0};
    SampleCallback callback_;
    std::atomic<bool> running_{false};
};

/**
 * @brief Factory for OrbbecCameraDriver (CameraBackendRegistry).
 * @param[in] id Sensor instance id from YAML.
 * @param[in] params Backend-specific key/value map.
 * @return Owning OrbbecCameraDriver*, or nullptr without OrbbecSDK.
 */
SensorDriver* CreateOrbbecCameraDriver(const SensorId& id,
                                       const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_CAMERA_ORBBEC_CAMERA_DRIVER_HPP_
