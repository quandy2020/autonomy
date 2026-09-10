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
 * @brief Stub chassis backend (software differential odom, no hardware).
 */

#ifndef AUTODRIVER_CHASSIS_STUB_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_STUB_DRIVER_HPP_

#include <memory>

#include "autodriver/chassis/chassis_driver.hpp"
#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace chassis {

std::shared_ptr<ChassisDriver> CreateStubChassisDriver(
    const ChassisId& id, const hardware::DriverParams& params);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_STUB_DRIVER_HPP_
