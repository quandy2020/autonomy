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
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"

namespace bolza {
namespace mobile_manipulator {

/**
 * Implementation of a wheel-based mobile manipulator.
 *
 * The wheel-based manipulator is simulated 2D-bicycle model for the base. The state
 * of the robot is: (base x, base y, base yaw, arm joints).
 *
 * The robot is assumed to be velocity controlled with the base commands as the forward
 * velocity and the angular velocity around z.
 */
class WheelBasedMobileManipulatorDynamics final : public SystemDynamicsBaseAD {
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
  WheelBasedMobileManipulatorDynamics(ManipulatorModelInfo modelInfo, const std::string& modelName,
                                      const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  ~WheelBasedMobileManipulatorDynamics() override = default;
  WheelBasedMobileManipulatorDynamics* Clone() const override { return new WheelBasedMobileManipulatorDynamics(*this); }

 private:
  WheelBasedMobileManipulatorDynamics(const WheelBasedMobileManipulatorDynamics& rhs) = default;

  ad_vector_t SystemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& /*parameters*/) const override;

  const ManipulatorModelInfo info_;
};

}  // namespace mobile_manipulator
}  // namespace bolza
