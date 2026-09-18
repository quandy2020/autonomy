/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

#include "autonomy/localization/atlas/runtime_config.hpp"
#include "autonomy/localization/atlas/sensor/camera/camera_sensor.hpp"
#include "autonomy/localization/atlas/sensor/imu/imu_sensor.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lidar_sensor.hpp"
#include "autonomy/localization/atlas/sensor/odom/odom_sensor.hpp"

#include <memory>

namespace autonomy::localization::atlas {
namespace sensor {

/**
 * Owns enabled IMU / Camera / Lidar / Odom sources for the single AtlasSystem.
 * Does not run a second SLAM or publish a fused pose authority.
 */
class SensorSuite {
public:
    explicit SensorSuite(RuntimeConfig config);

    bool Start();
    void Shutdown();

    [[nodiscard]] const RuntimeConfig& config() const { return config_; }
    [[nodiscard]] const common::ModalityFlags& flags() const { return config_.flags; }

    ImuSensor* imu() { return imu_.get(); }
    CameraSensor* camera() { return camera_.get(); }
    LidarSensor* lidar() { return lidar_.get(); }
    OdomSensor* odom() { return odom_.get(); }

private:
    RuntimeConfig config_;
    std::unique_ptr<ImuSensor> imu_;
    std::unique_ptr<CameraSensor> camera_;
    std::unique_ptr<LidarSensor> lidar_;
    std::unique_ptr<OdomSensor> odom_;
    bool running_ = false;
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
