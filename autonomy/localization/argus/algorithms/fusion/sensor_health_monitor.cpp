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

#include "argus/perception/sensor_health_monitor.hpp"

namespace argus {
namespace perception {

void SensorHealthMonitor::RecordLidar(double stamp_sec) {
    last_lidar_sec_ = stamp_sec;
}

void SensorHealthMonitor::RecordImu(double stamp_sec) {
    last_imu_sec_ = stamp_sec;
}

void SensorHealthMonitor::RecordCamera(double stamp_sec) {
    last_camera_sec_ = stamp_sec;
}

SensorHealth SensorHealthMonitor::Evaluate(double now_sec) const {
    SensorHealth health;
    const auto AgeOf = [&](double last_stamp_sec) {
        return (last_stamp_sec < 0.0) ? timeout_sec_ + 1.0
                                      : (now_sec - last_stamp_sec);
    };
    health.lidar_age_sec = AgeOf(last_lidar_sec_);
    health.imu_age_sec = AgeOf(last_imu_sec_);
    health.camera_age_sec = AgeOf(last_camera_sec_);
    health.lidar_healthy = health.lidar_age_sec <= timeout_sec_;
    health.imu_healthy = health.imu_age_sec <= timeout_sec_;
    health.camera_healthy = health.camera_age_sec <= timeout_sec_;
    return health;
}

}  // namespace perception
}  // namespace argus
