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
 * @file centroidal_model_pinocchio_mapping.hpp
 * @brief Centroidal Dynamics:.
 */


#include "bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp"

#include "bolza/pinocchio/centroidal/centroidal_model_info.hpp"

namespace bolza {

template <typename SCALAR>
class CentroidalModelPinocchioMappingTpl;

using CentroidalModelPinocchioMapping = CentroidalModelPinocchioMappingTpl<scalar_t>;
using CentroidalModelPinocchioMappingCppAd = CentroidalModelPinocchioMappingTpl<ad_scalar_t>;

/**
 * Centroidal Dynamics:
 *
 * State: x = [ linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_positions ]'
 * @remark: The linear and angular momenta are expressed with respect to the centroidal frame (a frame centered at
 * the CoM and aligned with the inertial frame).
 *
 * Input: u = [ contact_forces, contact_wrenches, joint_velocities ]'
 * @remark: Contact forces and wrenches are expressed with respect to the inertial frame.
 *
 *
 * Pinocchio Joint Positions: qPinocchio = [ base_position, base_orientation_zyx, joint_positions ]'
 * @remark: Base position is expressed with respect to the inertial frame
 *
 * Pinocchio Joint Velocities: vPinocchio = [ base_linear_velocity, base_orientation_zyx_derivatives, joint_velocities ]'
 * @remark: Base linear velocity is expressed with respect to the inertial frame
 */
template <typename SCALAR>
class CentroidalModelPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  /**
   * Constructor
   * @param [in] centroidalModelInfo: centroidal model information.
   */
  explicit CentroidalModelPinocchioMappingTpl(CentroidalModelInfoTpl<SCALAR> centroidalModelInfo);

  /**
   * @brief Destructor.
   */
  ~CentroidalModelPinocchioMappingTpl() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  CentroidalModelPinocchioMappingTpl* Clone() const override;

  /** Sets the pinocchio interface for caching
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   * @note The pinocchio interface must be set before calling the getters.
   */
  void SetPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) override;

  /**
   * Computes the vector of generalized coordinates (qPinocchio) used by pinocchio functions from the robot state variables
   *
   * @param [in] state: system state vector
   * @return pinocchio joint positions, which are also the robot's generalized positions with a ZYX-Euler angle
   * parameterization of the base orientation
   */
  vector_t GetPinocchioJointPosition(const vector_t& state) const override;

  /**
   * Computes the vector of generalized velocities (vPinocchio) used by pinocchio functions from the robot state and input variables
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return pinocchio joint velocities, which are also the time derivatives of the pinocchio joint positions
   *
   * @note requires pinocchioInterface to be updated with:
   *       bolza::UpdateCentroidalDynamics(interface, info, q)
   */
  vector_t GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * Maps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   * @param [in] state: system state vector
   * @param [in] Jq: jacobian with respect to pinocchio joint positions
   * @param [in] Jv: jacobian with respect to pinocchio joint velocities
   * @return a pair {dfdx, dfdu} containing the jacobians with respect to the system state and input
   *
   * @note requires pinocchioInterface to be updated with:
   *       bolza::UpdateCentroidalDynamicsDerivatives(interface, info, q, v)
   *
   * TODO: Add Jacobian w.r.t generalized accelerations as argument to get a full implicit dependence on the inputs
   */
  std::pair<matrix_t, matrix_t> GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  /**
   * @brief Returns a structure containing robot-specific information needed for the centroidal dynamics computations.
   * @return Result value.
   */
  const CentroidalModelInfoTpl<SCALAR>& GetCentroidalModelInfo() const { return centroidalModelInfo_; }

 private:
  /**
   * @brief Constructs a CentroidalModelPinocchioMappingTpl.
   * @param [in] rhs: Rhs.
   */
  CentroidalModelPinocchioMappingTpl(const CentroidalModelPinocchioMappingTpl& rhs);

  const PinocchioInterfaceTpl<SCALAR>* pinocchioInterfacePtr_;
  const CentroidalModelInfoTpl<SCALAR> centroidalModelInfo_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class CentroidalModelPinocchioMappingTpl<scalar_t>;
extern template class CentroidalModelPinocchioMappingTpl<ad_scalar_t>;

}  // namespace bolza
