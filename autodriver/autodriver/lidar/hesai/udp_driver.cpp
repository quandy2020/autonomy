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

#include "autodriver/lidar/hesai/udp_driver.hpp"

#include "autodriver/lidar/backend_register.hpp"

namespace autodriver {
namespace hardware {

SensorDriver* CreateHesaiUdpDriver(const SensorId& id,
                                   const DriverParams& params) {
    return new HesaiUdpDriver(id, params);
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_LIDAR_BACKEND(hesai, "hesai",
                       autodriver::hardware::CreateHesaiUdpDriver, "pandar");
