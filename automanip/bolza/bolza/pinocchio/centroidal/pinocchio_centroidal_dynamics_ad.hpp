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

/**
 * @file pinocchio_centroidal_dynamics_ad.hpp
 * @brief Centroidal Dynamics:.
 */


#include <memory>
#include <string>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include "bolza/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp"

namespace bolza {

/**
 * Centroidal Dynamics:
 *
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * @remark: The linear and angular momenta are expressed with respect to the centroidal frame (a frame centered at
 * the CoM and aligned with the inertial frame).
 *
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
 */
class PinocchioCentroidalDynamicsAD final {
 public:
  /**
   * @brief Constructor.
   *
   * available.
   * @param [in] pinocchioInterface: The pinocchio interface.
   * @param [in] info: Info.
   * @param [in] modelName: Name of the generate model library
   * @param [in] modelFolder: Folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   * @param [in] verbose: print information.
   */
  PinocchioCentroidalDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info, const std::string& modelName,
                                const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = false);

  /**
   * @brief Copy Constructor.
   * @param [in] rhs: Rhs.
   */
  PinocchioCentroidalDynamicsAD(const PinocchioCentroidalDynamicsAD& rhs);

  /**
   * Computes system flow map x_dot = f(x, u)
   *
   * @param [in] time: time
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return system flow map x_dot = f(x, u)
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input) const;

  /**
   * Computes first order approximation of the system flow map x_dot = f(x, u)
   *
   * @param [in] time: time
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return linear approximation of system flow map x_dot = f(x, u)
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const;

 private:
  /**
   * @brief Get Value Cpp Ad.
   * @param [in,out] pinocchioInterfaceCppAd: Pinocchio Interface Cpp Ad.
   * @param [in] mapping: Mapping.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  ad_vector_t GetValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const CentroidalModelPinocchioMappingCppAd& mapping,
                            const ad_vector_t& state, const ad_vector_t& input);

  std::unique_ptr<CppAdInterface> systemFlowMapCppAdInterfacePtr_;
};

}  // namespace bolza
