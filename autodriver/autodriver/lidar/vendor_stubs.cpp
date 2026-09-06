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
 * Covers livox, rslidar, lslidar, seyond, vanjeelidar.
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

std::shared_ptr<SensorDriver> MakeStub(const char* vendor, const SensorId& id) {
    AERROR << vendor << " lidar backend not implemented (id=" << id
           << "); implement lidar/" << vendor << " backend";
    return nullptr;
}

}  // namespace

std::shared_ptr<SensorDriver> CreateLivoxLidarDriver(
    const SensorId& id, const DriverParams&) {
    return MakeStub("livox", id);
}

std::shared_ptr<SensorDriver> CreateRsLidarDriver(const SensorId& id,
                                                  const DriverParams&) {
    return MakeStub("rslidar", id);
}

std::shared_ptr<SensorDriver> CreateLsLidarDriver(const SensorId& id,
                                                  const DriverParams&) {
    return MakeStub("lslidar", id);
}

std::shared_ptr<SensorDriver> CreateSeyondLidarDriver(const SensorId& id,
                                                      const DriverParams&) {
    return MakeStub("seyond", id);
}

std::shared_ptr<SensorDriver> CreateVanjeeLidarDriver(const SensorId& id,
                                                      const DriverParams&) {
    return MakeStub("vanjee", id);
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_LIDAR_BACKEND(livox, "livox",
                       autodriver::hardware::CreateLivoxLidarDriver);
REGISTER_LIDAR_BACKEND(rslidar, "rslidar",
                       autodriver::hardware::CreateRsLidarDriver, "robosense");
REGISTER_LIDAR_BACKEND(lslidar, "lslidar",
                       autodriver::hardware::CreateLsLidarDriver);
REGISTER_LIDAR_BACKEND(seyond, "seyond",
                       autodriver::hardware::CreateSeyondLidarDriver);
REGISTER_LIDAR_BACKEND(vanjee, "vanjee",
                       autodriver::hardware::CreateVanjeeLidarDriver,
                       "vanjeelidar");
