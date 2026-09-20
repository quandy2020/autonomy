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
 * @file driver.hpp
 * @brief JetAuto ChassisDriver (RRC serial + mecanum/differential IK).
 */

#ifndef AUTODRIVER_CHASSIS_JETAUTO_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_JETAUTO_DRIVER_HPP_

#include "chassis/chassis_driver.hpp"
#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief Create JetAuto ChassisDriver (backend "jetauto", alias "hiwonder").
 *
 * Params (YAML chassis.params / port shorthand):
 *   device|port, baud (default 1000000), drive_mode (mecanum|differential),
 *   wheelbase, track_width, wheel_diameter, max_rps, simulate (skip serial).
 *
 * @param[in] id Chassis instance id.
 * @param[in] params Backend key/value map.
 * @return Owning ChassisDriver* (never null).
 */
ChassisDriver* CreateJetAutoChassisDriver(
    const ChassisId& id, const hardware::DriverParams& params);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_JETAUTO_DRIVER_HPP_
