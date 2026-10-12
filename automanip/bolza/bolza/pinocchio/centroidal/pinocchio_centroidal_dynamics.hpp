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
 * @file pinocchio_centroidal_dynamics.hpp
 * @brief Centroidal Dynamics:.
 */


#include <bolza/core/types.hpp>
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
class PinocchioCentroidalDynamics final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
  using Matrix3x = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

  /**
   * @brief Constructor.
   * @param [in] info: Info.
   */
  explicit PinocchioCentroidalDynamics(CentroidalModelInfo info);

  /**
   * @brief Copy Constructor.
   * @param [in] rhs: Rhs.
   */
  PinocchioCentroidalDynamics(const PinocchioCentroidalDynamics& rhs);

  /** Set the pinocchio interface for caching.
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   * @note The pinocchio interface must be set before calling the getters.
   */
  void SetPinocchioInterface(const PinocchioInterface& pinocchioInterface) {
    pinocchioInterfacePtr_ = &pinocchioInterface;
    /**
     * @brief Set Pinocchio Interface.
     * @param [in] pinocchioInterface: Pinocchio Interface.
     * @return Result value.
     */
    mapping_.SetPinocchioInterface(pinocchioInterface);
  }

  /**
   * Computes system flow map x_dot = f(x, u)
   *
   * @param [in] time: time
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return system flow map x_dot = f(x, u)
   *
   * @note requires pinocchioInterface to be updated with:
   *       bolza::UpdateCentroidalDynamics(interface, info, q)
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input);

  /**
   * Computes first order approximation of the system flow map x_dot = f(x, u)
   *
   * @param [in] time: time
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return linear approximation of system flow map x_dot = f(x, u)
   *
   * @note requires pinocchioInterface to be updated with:
   *       bolza::UpdateCentroidalDynamicsDerivatives(interface, info, q, v)
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input);

 private:
  /**
   * Computes the gradients of the normalized centroidal momentum rate (linear + angular) expressed in the centroidal frame
   *
   * @param [in] state: system state vector
   * @param [in] input: system input vector
   * @return time derivative of normalized centroidal momentum (required for the linear approximation)
   */
  void ComputeNormalizedCentroidalMomentumRateGradients(const vector_t& state, const vector_t& input);

  const PinocchioInterface* pinocchioInterfacePtr_;
  CentroidalModelPinocchioMapping mapping_;

  // partial derivatives of the system dynamics
  Matrix3x normalizedLinearMomentumRateDerivativeQ_;
  Matrix3x normalizedAngularMomentumRateDerivativeQ_;
  Matrix3x normalizedLinearMomentumRateDerivativeInput_;
  Matrix3x normalizedAngularMomentumRateDerivativeInput_;
};
}  // namespace bolza
