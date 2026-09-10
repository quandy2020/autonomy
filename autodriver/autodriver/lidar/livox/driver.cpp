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
 * @brief Livox lidar_3d factory — selects SDK1 or SDK2 by model/sdk param.
 */

#include <memory>
#include <string>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/backend_register.hpp"
#include "autodriver/lidar/livox/model.hpp"
#include "autodriver/lidar/livox/sdk1_driver.hpp"
#include "autodriver/lidar/livox/sdk2_driver.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace hardware {

std::shared_ptr<SensorDriver> CreateLivoxLidarDriver(
    const SensorId& id, const DriverParams& params) {
    const std::string model = GetString(params, "model", "Mid-360");
    const std::string sdk = GetString(params, "sdk", "");
    if (lidar::livox::UsesSdk2(model, sdk)) {
        AINFO << "Livox using SDK2 for id=" << id << " model=" << model;
        return CreateLivoxSdk2Driver(id, params);
    }
    AINFO << "Livox using SDK1 for id=" << id << " model=" << model;
    return CreateLivoxSdk1Driver(id, params);
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_LIDAR_BACKEND(livox, "livox",
                       autodriver::hardware::CreateLivoxLidarDriver);
