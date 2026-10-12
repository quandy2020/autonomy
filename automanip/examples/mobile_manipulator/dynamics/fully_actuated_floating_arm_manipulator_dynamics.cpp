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

#include "examples/mobile_manipulator/dynamics/fully_actuated_floating_arm_manipulator_dynamics.hpp"

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FullyActuatedFloatingArmManipulatorDynamics::FullyActuatedFloatingArmManipulatorDynamics(const ManipulatorModelInfo& info,
                                                                                         const std::string& modelName,
                                                                                         const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                                                         bool recompileLibraries /*= true*/,
                                                                                         bool verbose /*= true*/) {
  this->Initialize(info.stateDim, info.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t FullyActuatedFloatingArmManipulatorDynamics::SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                                       const ad_vector_t&) const {
  return input;
}

}  // namespace mobile_manipulator
}  // namespace bolza
