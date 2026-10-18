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

/**
 * @file sensor_health_monitor.hpp
 * @brief Per-sensor freshness tracking for confidence gating.
 */

#ifndef ARGUS_PERCEPTION_SENSOR_HEALTH_MONITOR_HPP_
#define ARGUS_PERCEPTION_SENSOR_HEALTH_MONITOR_HPP_

namespace argus {
namespace perception {

/**
 * @brief Snapshot of LiDAR / IMU / camera health at one evaluate time.
 */
struct SensorHealth {
    /** @brief LiDAR age within timeout. */
    bool lidar_healthy = true;
    /** @brief IMU age within timeout. */
    bool imu_healthy = true;
    /** @brief Camera age within timeout. */
    bool camera_healthy = true;
    /** @brief now - last LiDAR stamp (s). */
    double lidar_age_sec = 0.0;
    /** @brief now - last IMU stamp (s). */
    double imu_age_sec = 0.0;
    /** @brief now - last camera stamp (s). */
    double camera_age_sec = 0.0;
};

/**
 * @brief Tracks per-sensor freshness / basic sanity for gating.
 */
class SensorHealthMonitor
{
public:
    /**
     * @brief Maximum age before a sensor is marked unhealthy.
     * @param timeout_sec Timeout in seconds.
     */
    void SetTimeoutSec(double timeout_sec) {
        timeout_sec_ = timeout_sec;
    }

    /**
     * @brief Record a LiDAR measurement time.
     * @param stamp_sec Measurement timestamp (s).
     */
    void RecordLidar(double stamp_sec);

    /**
     * @brief Record an IMU measurement time.
     * @param stamp_sec Measurement timestamp (s).
     */
    void RecordImu(double stamp_sec);

    /**
     * @brief Record a camera measurement time.
     * @param stamp_sec Measurement timestamp (s).
     */
    void RecordCamera(double stamp_sec);

    /**
     * @brief Evaluate ages relative to `now_sec`.
     * @param now_sec Reference time (usually current package stamp).
     * @return Per-sensor healthy flags and ages.
     */
    SensorHealth Evaluate(double now_sec) const;

private:
    double timeout_sec_ = 0.5;
    double last_lidar_sec_ = -1.0;
    double last_imu_sec_ = -1.0;
    double last_camera_sec_ = -1.0;
};

}  // namespace perception
}  // namespace argus

#endif  // ARGUS_PERCEPTION_SENSOR_HEALTH_MONITOR_HPP_
