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
 * @file centroidal_model_rbd_conversions.hpp
 * @brief Constructor.
 */


#include <bolza/core/types.hpp>
#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include "bolza/pinocchio/centroidal/centroidal_model_pinocchio_mapping.hpp"

namespace bolza {

class CentroidalModelRbdConversions final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix3 = Eigen::Matrix<scalar_t, 3, 3>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

  /**
   * @brief Constructor.
   * @param [in] pinocchioInterface: predefined pinocchio interface for a robot
   * @param [in] info: Info.
   */
  CentroidalModelRbdConversions(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info);

  /**
   * Computes the floating-base generalized positions, velocities, and accelerations.
   *
   * @param [in] state: ocs2 switched-model state vector
   * @param [in] input: ocs2 switched-model input vector
   * @param [in] jointAccelerations: actuated joints accelerations
   * @param [out] basePose: [base position, base orientation (EulerAngles-ZYX)] expressed in the world frame
   * @param [out] baseVelocity: [base linear velocity, base angular velocity] expressed in the world frame
   * @param [out] baseAcceleration: [base linear acceleration, base angular acceleration] expressed in the world frame
   */
  void ComputeBaseKinematicsFromCentroidalModel(const vector_t& state, const vector_t& input, const vector_t& jointAccelerations,
                                                Vector6& basePose, Vector6& baseVelocity, Vector6& baseAcceleration);

  /**
   * Computes the ocs2 centroidal model state from the rigid body dynamics model state.
   * @note: In rbdState, orientations precede positions and angular velocities precede linear velocities
   * @note: The input rbdState contains non-local base velocities (expressed in world frame)
   *
   * @param [in] rbdState: rigid body dynamics model state [base pose, joint positions, base twist, joint velocities]
   * @return ocs2 switched-model state vector
   */
  vector_t ComputeCentroidalStateFromRbdModel(const vector_t& rbdState);

  /**
   * Computes the rigid body dynamics model state from the ocs2 centroidal model state
   * @note: The output rbdState contains non-local base velocities (expressed in world frame)
   *
   * @param [in] state: ocs2 switched-model state vector
   * @param [in] input: ocs2 switched-model input vector
   * @return rigid body dynamics model state [base pose, joint positions, base twist, joint velocities]
   */
  vector_t ComputeRbdStateFromCentroidalModel(const vector_t& state, const vector_t& input);

  /**
   * Computes the rigid body dynamics model torque from the ocs2 centroidal model input
   * @note: Calls ComputeRbdTorqueFromCentroidalModelPD() with zero gains, so only returning the feedforward torque
   *
   * @param [in] state: ocs2 switched-model state vector
   * @param [in] input: ocs2 switched-model input vector
   * @param [in] jointAccelerations: actuated joints accelerations
   * @return rigid body dynamics model torque [base wrench, joint torques]
   */
  vector_t ComputeRbdTorqueFromCentroidalModel(const vector_t& state, const vector_t& input, const vector_t& jointAccelerations);

  /**
   * Computes the rigid body dynamics model torque from the ocs2 centroidal model input and adds PD feedback
   * @note: Calls ComputeRbdTorqueFromCentroidalModelPD() with gains loaded from settings
   *
   * @param [in] desiredState: desired ocs2 switched-model state vector
   * @param [in] desiredInput: desired ocs2 switched-model input vector
   * @param [in] desiredJointAccelerations: desired joint accelerations
   * @param [in] measuredRbdState: measured rigid body dynamics model state (required for PD control)
   * @return rigid body dynamics model torque [base wrench, joint torques]
   */
  vector_t ComputeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                 const vector_t& desiredJointAccelerations, const vector_t& measuredRbdState);

  /**
   * Computes the rigid body dynamics model torque from the ocs2 centroidal model input and adds PD feedback
   * @note: PD controller is added on the acceleration level
   *
   * @param [in] desiredState: desired ocs2 switched-model state vector
   * @param [in] desiredInput: desired ocs2 switched-model input vector
   * @param [in] desiredJointAccelerations: desired joint accelerations
   * @param [in] measuredRbdState: measured rigid body dynamics model state (required for PD control)
   * @param [in] pGains: proportional gains for [base, joint] tracking
   * @param [in] dGains: derivative gains for [base, joint] tracking
   * @return rigid body dynamics model torque [base wrench, joint torques]
   */
  vector_t ComputeRbdTorqueFromCentroidalModelPD(const vector_t& desiredState, const vector_t& desiredInput,
                                                 const vector_t& desiredJointAccelerations, const vector_t& measuredRbdState,
                                                 const vector_t& pGains, const vector_t& dGains);

  /**
   * Load settings from a configuration file
   *
   * @param [in] fileName: File name which contains the configuration data
   * @param [in] fieldName: Field name which contains the configuration data
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not
   */
  void LoadSettings(const std::string& fileName, const std::string& fieldName, bool verbose = true);

 private:
  /**
   * @brief Constructs a CentroidalModelRbdConversions.
   * @param [in] other: Other.
   */
  CentroidalModelRbdConversions(const CentroidalModelRbdConversions& other) = default;
  CentroidalModelRbdConversions& operator=(const CentroidalModelRbdConversions& rhs) = default;

  PinocchioInterface pinocchioInterface_;
  CentroidalModelPinocchioMapping mapping_;

  // settings
  vector_t pGains_;
  vector_t dGains_;
};

}  // namespace bolza
