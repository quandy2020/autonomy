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
 * @brief Conti ARS-style radar driver factory (skeleton).
 */

#ifndef AUTODRIVER_RADAR_CONTI_DRIVER_HPP_
#define AUTODRIVER_RADAR_CONTI_DRIVER_HPP_

#include <memory>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Create a Conti CAN radar driver, or nullptr until ProtocolData lands.
 *
 * Planned path: `canbus::CanReceiver` + Conti object-list ProtocolData →
 * `RadarSample` (currently PointCloud2 placeholder). YAML: `backend: conti`.
 *
 * @param id Sensor id (e.g. radar/front).
 * @param params Expect `interface` / CAN ids when implemented.
 */
std::shared_ptr<SensorDriver> CreateContiRadarDriver(
    const SensorId& id, const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_CONTI_DRIVER_HPP_
