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
    OrbbecCameraDriver(SensorId id, DriverParams params);
    ~OrbbecCameraDriver() override;

    SensorType GetType() const override { return SensorType::kCamera; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

private:
    SensorId id_;
    DriverParams params_;
    orbbec::StreamKind stream_{orbbec::StreamKind::kColor};
    int width_{640};
    int height_{480};
    int fps_{30};
    std::shared_ptr<io::OrbbecDeviceHub> hub_;
    std::uint64_t subscription_id_{0};
    SampleCallback callback_;
    std::atomic<bool> running_{false};
};

/**
 * @brief Factory for OrbbecCameraDriver (CameraBackendRegistry).
 */
std::shared_ptr<SensorDriver> CreateOrbbecCameraDriver(
    const SensorId& id, const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_CAMERA_ORBBEC_CAMERA_DRIVER_HPP_
