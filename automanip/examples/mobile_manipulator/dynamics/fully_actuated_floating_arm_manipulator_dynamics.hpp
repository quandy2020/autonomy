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

#pragma once

#include <bolza/core/dynamics/system_dynamics_base_ad.hpp>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>

namespace bolza {
namespace mobile_manipulator {

/**
 * Implementation of dynamis for a free floating arm manipulator with actuatable base.
 *
 * The floating base manipulator has the state: (6-DOF base pose, arm joints).
 * The base orientation is represented using zyx euler angles.
 * The end-effector targets are assumed to given with respect to the world frame.
 * The arm is assumed to be velocity controlled.
 */
class FullyActuatedFloatingArmManipulatorDynamics final : public SystemDynamicsBaseAD {
 public:
  /**
   * Constructor
   *
   * @param [in] modelInfo : The manipulator information.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, always compile the model library, else try to load existing library if available.
   * @param [in] verbose : Display information.
   */
  FullyActuatedFloatingArmManipulatorDynamics(const ManipulatorModelInfo& modelInfo, const std::string& modelName,
                                              const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                                              bool verbose = true);

  ~FullyActuatedFloatingArmManipulatorDynamics() override = default;
  FullyActuatedFloatingArmManipulatorDynamics* Clone() const override { return new FullyActuatedFloatingArmManipulatorDynamics(*this); }

 private:
  FullyActuatedFloatingArmManipulatorDynamics(const FullyActuatedFloatingArmManipulatorDynamics& rhs) = default;

  ad_vector_t SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& /*parameters*/) const override;
};

}  // namespace mobile_manipulator
}  // namespace bolza
