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

#include <bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"

namespace bolza {
namespace mobile_manipulator {

template <typename SCALAR>
class MobileManipulatorPinocchioMappingTpl;

using MobileManipulatorPinocchioMapping = MobileManipulatorPinocchioMappingTpl<scalar_t>;
using MobileManipulatorPinocchioMappingCppAd = MobileManipulatorPinocchioMappingTpl<ad_scalar_t>;

/**
 * Pinocchio state and input mapping.
 */
template <typename SCALAR>
class MobileManipulatorPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  /**
   * Constructor
   * @param [in] info : mobile manipulator model information.
   */
  explicit MobileManipulatorPinocchioMappingTpl(ManipulatorModelInfo info);

  ~MobileManipulatorPinocchioMappingTpl() override = default;
  MobileManipulatorPinocchioMappingTpl<SCALAR>* Clone() const override;

  /**
   * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state.
   *
   * @param [in] state: system state vector
   * @return pinocchio joint positions, which are also the robot's generalized positions with a ZYX-Euler angle
   * parameterization of the base orientation
   */
  vector_t GetPinocchioJointPosition(const vector_t& state) const override;

  /**
   * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input.
   *
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return pinocchio joint velocities, which are also the time derivatives of the pinocchio joint positions
   */
  vector_t GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   *
   * @param [in] state: system state vector
   * @param [in] Jq: jacobian with respect to pinocchio joint positions
   * @param [in] Jv: jacobian with respect to pinocchio joint velocities
   * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
   */
  std::pair<matrix_t, matrix_t> GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /**
   * Returns the mobile manipulator model info.
   */
  const ManipulatorModelInfo& GetManipulatorModelInfo() const { return modelInfo_; }

 private:
  MobileManipulatorPinocchioMappingTpl(const MobileManipulatorPinocchioMappingTpl& rhs) = default;

  const ManipulatorModelInfo modelInfo_;
};

}  // namespace mobile_manipulator
}  // namespace bolza
