/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @brief Stub arm backend: integrates joint commands, no controller.
 */

#ifndef AUTOMANIP_ARM_STUB_DRIVER_HPP_
#define AUTOMANIP_ARM_STUB_DRIVER_HPP_

#include "arm/arm_driver.hpp"
#include "arm/plant.hpp"

namespace automanip {
namespace arm {

ArmDriver* CreateStubArmDriver(const std::string& id,
                               const ArmPlantOptions& options);

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_STUB_DRIVER_HPP_
