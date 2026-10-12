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
 * @file pinocchio_end_effector_kinematics.hpp
 * @brief End-effector Kinematics implementation using pinocchio.
 */


#include <string>
#include <vector>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"
#include "bolza/pinocchio/interface/pinocchio_state_input_mapping.hpp"
#include <bolza/tools/end_effector/end_effector_kinematics.hpp>

namespace bolza {

/**
 * End-effector Kinematics implementation using pinocchio.
 *
 * This class uses caching of computation done on the pinocchio::Data in PinocchiInterface.
 * See in the method documentation which pinocchio functions are required to update pinocchio::Data.
 *
 * Example:
 *   PinocchioEndEffectorKinematics kinematics(pinocchioInterface, mapping, {"END_EFFECTOR_NAME"});
 *   pinocchio::forwardKinematics(pinocchioInterface.GetModel(), pinocchioInterface.GetData(), q);
 *   pinocchio::updateFramePlacements(pinocchioInterface.GetModel(), pinocchioInterface.GetData());
 *   kinematics.SetPinocchioInterface(pinocchioInterface);
 *   const auto pos = kinematics.GetPosition(x);
 */
class PinocchioEndEffectorKinematics final : public EndEffectorKinematics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;

  /** Constructor
   * @param [in] pinocchioInterface: pinocchio interface.
   * @param [in] mapping: mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds: array of end effector names.
   */
  PinocchioEndEffectorKinematics(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping,
                                 std::vector<std::string> endEffectorIds);

  /**
   * @brief Destructor.
   */
  ~PinocchioEndEffectorKinematics() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  PinocchioEndEffectorKinematics* Clone() const override;
  PinocchioEndEffectorKinematics& operator=(const PinocchioEndEffectorKinematics&) = delete;

  /** Set the pinocchio interface for caching.
   * @note The pinocchio interface must be set before calling the getters.
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   */
  void SetPinocchioInterface(const PinocchioInterface& pinocchioInterface) {
    pinocchioInterfacePtr_ = &pinocchioInterface;
    /**
     * @brief Set Pinocchio Interface.
     * @param [in] pinocchioInterface: Pinocchio Interface.
     * @return Result value.
     */
    mappingPtr_->SetPinocchioInterface(pinocchioInterface);
  }

  /**
   * @brief Get end-effector IDs (names).
   * @return Result value.
   */
  const std::vector<std::string>& GetIds() const override;

  /**
   * @brief Get the end effector position vectors.
   *
   * pinocchio::forwardKinematics(model, data, q)
   *
   * pinocchio::updateFramePlacements(model, data)
   * @note requires pinocchioInterface to be updated with:
   * @param [in] state: State.
   * @return Result value.
   */
  std::vector<vector3_t> GetPosition(const vector_t& state) const override;

  /**
   * @brief Get the end effector velocity vectors.
   *
   * pinocchio::forwardKinematics(model, data, q, v)
   * @note requires pinocchioInterface to be updated with:
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  std::vector<vector3_t> GetVelocity(const vector_t& state, const vector_t& input) const override;

  /**
   * @brief Get the end effector orientation error.
   *
   * pinocchio::forwardKinematics(model, data, q)
   *
   * pinocchio::updateFramePlacements(model, data)
   * @note requires pinocchioInterface to be updated with:
   * @param [in] state: State.
   * @param [in] referenceOrientations: Reference Orientations.
   * @return Result value.
   */
  std::vector<vector3_t> GetOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

  /**
   * @brief Get the end effector position linear approximation.
   *
   * pinocchio::forwardKinematics(model, data, q)
   *
   * pinocchio::updateFramePlacements(model, data)
   *
   * pinocchio::computeJointJacobians(model, data)
   * @note requires pinocchioInterface to be updated with:
   * @param [in] state: State.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetPositionLinearApproximation(const vector_t& state) const override;

  /**
   * @brief Get the end effector velocity linear approximation.
   *
   * pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a)
   * @note requires pinocchioInterface to be updated with:
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetVelocityLinearApproximation(const vector_t& state,
                                                                                const vector_t& input) const override;

  /**
   * @brief Get the end effector orientation error linear approximation.
   *
   * pinocchio::forwardKinematics(model, data, q)
   *
   * pinocchio::updateFramePlacements(model, data)
   *
   * pinocchio::computeJointJacobians(model, data)
   * @note requires pinocchioInterface to be updated with:
   * @param [in] state: State.
   * @param [in] referenceOrientations: Reference Orientations.
   * @return Result value.
   */
  std::vector<VectorFunctionLinearApproximation> GetOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

 private:
  /**
   * @brief Constructs a PinocchioEndEffectorKinematics.
   * @param [in] rhs: Rhs.
   */
  PinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics& rhs);

  const PinocchioInterface* pinocchioInterfacePtr_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
  const std::vector<std::string> endEffectorIds_;
  std::vector<size_t> endEffectorFrameIds_;
};

}  // namespace bolza
