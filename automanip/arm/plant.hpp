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
 * @file plant.hpp
 * @brief Options handed to an arm backend factory.
 */

#ifndef AUTOMANIP_ARM_PLANT_HPP_
#define AUTOMANIP_ARM_PLANT_HPP_

#include <string>

#include "arm/chain.hpp"
#include "arm/osc2/controller.hpp"

namespace automanip {
namespace arm {

/**
 * @brief Kinematic plant description shared by every backend.
 */
struct ArmPlantOptions {
  std::string frame_id = "arm_base";
  std::string tool_frame_id = "tool0";
  SerialChain chain = MakeDefaultArm();
  osc2::Osc2Settings osc2;
};

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_PLANT_HPP_
