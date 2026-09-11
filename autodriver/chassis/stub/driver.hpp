/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file driver.hpp
 * @brief Stub chassis backend (software differential odom, no hardware).
 */

#ifndef AUTODRIVER_CHASSIS_STUB_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_STUB_DRIVER_HPP_

#include <memory>

#include "chassis/chassis_driver.hpp"
#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief Create the in-process stub ChassisDriver (backend name "stub").
 *
 * Integrates differential-drive odometry from ApplyVelocityCommand; used for
 * bring-up and CI without real hardware. Also registered under aliases
 * "sim" / "fake".
 * @param[in] id Chassis instance id from YAML (e.g. "chassis/base").
 * @param[in] params Optional keys such as battery_soc in [0, 1].
 * @return Owning raw ChassisDriver* for NamedProductFactory / never null.
 */
ChassisDriver* CreateStubChassisDriver(
    const ChassisId& id, const hardware::DriverParams& params);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_STUB_DRIVER_HPP_
