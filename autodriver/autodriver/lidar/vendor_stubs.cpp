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
 * @brief Vendor lidar backends registered as stubs (Create → nullptr).
 *
 * Covers rslidar, lslidar, seyond, vanjeelidar.
 * Livox is implemented under lidar/livox/ (overwrites if both link).
 * Fusion (pri/sec) is not a Lidar3d backend — use SensorHub / a future fusion
 * module instead.
 */

#include <memory>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/backend_register.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace hardware {
namespace {

/**
 * @brief Log and return nullptr for an unimplemented lidar vendor backend.
 * @param vendor Canonical vendor name used in the error message.
 * @param id Sensor instance id from YAML.
 * @return Always nullptr.
 */
SensorDriver* MakeStub(const char* vendor, const SensorId& id) {
    AERROR << vendor << " lidar backend not implemented (id=" << id
           << "); implement lidar/" << vendor << " backend";
    return nullptr;
}

}  // namespace

/** @brief Stub factory for RoboSense (`rslidar` / alias `robosense`). */
SensorDriver*
CreateRsLidarDriver(const SensorId& id,
                                                  const DriverParams&) {
    return MakeStub("rslidar", id);
}

/** @brief Stub factory for LSLidar (`lslidar`). */
SensorDriver*
CreateLsLidarDriver(const SensorId& id,
                                                  const DriverParams&) {
    return MakeStub("lslidar", id);
}

/** @brief Stub factory for Seyond (`seyond`). */
SensorDriver*
CreateSeyondLidarDriver(const SensorId& id,
                                                      const DriverParams&) {
    return MakeStub("seyond", id);
}

/** @brief Stub factory for Vanjee (`vanjee` / alias `vanjeelidar`). */
SensorDriver*
CreateVanjeeLidarDriver(const SensorId& id,
                                                      const DriverParams&) {
    return MakeStub("vanjee", id);
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_LIDAR_BACKEND(rslidar, "rslidar",
                       autodriver::hardware::CreateRsLidarDriver, "robosense");
REGISTER_LIDAR_BACKEND(lslidar, "lslidar",
                       autodriver::hardware::CreateLsLidarDriver, "");
REGISTER_LIDAR_BACKEND(seyond, "seyond",
                       autodriver::hardware::CreateSeyondLidarDriver, "");
REGISTER_LIDAR_BACKEND(vanjee, "vanjee",
                       autodriver::hardware::CreateVanjeeLidarDriver,
                       "vanjeelidar");
