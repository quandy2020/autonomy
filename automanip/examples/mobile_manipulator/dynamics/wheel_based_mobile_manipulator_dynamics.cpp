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

#include "examples/mobile_manipulator/dynamics/wheel_based_mobile_manipulator_dynamics.hpp"

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WheelBasedMobileManipulatorDynamics::WheelBasedMobileManipulatorDynamics(ManipulatorModelInfo info, const std::string& modelName,
                                                                         const std::string& modelFolder /*= "/tmp/ocs2"*/,
                                                                         bool recompileLibraries /*= true*/, bool verbose /*= true*/)
    : info_(std::move(info)) {
  this->Initialize(info_.stateDim, info_.inputDim, modelName, modelFolder, recompileLibraries, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t WheelBasedMobileManipulatorDynamics::SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                                               const ad_vector_t&) const {
  ad_vector_t Dxdt(info_.stateDim);
  const auto theta = state(2);
  const auto v = Input(0);  // forward velocity in base frame
  Dxdt << cos(theta) * v, sin(theta) * v, Input(1), input.tail(info_.armDim);
  return dxdt;
}

}  // namespace mobile_manipulator
}  // namespace bolza
