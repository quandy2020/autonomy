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
 * @file end_effector_kinematics.hpp
 * @brief The Kinematics function which maps state-input pair to the end-effector (position, velocity, orientation error).
 */


#include <string>
#include <utility>

#include <bolza/core/types.hpp>

namespace bolza {

/** The Kinematics function which maps state-input pair to the end-effector (position, velocity, orientation error) */
template <typename SCALAR_T>
class EndEffectorKinematics {
 public:
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  using quaternion_t = Eigen::Quaternion<SCALAR_T>;

  /**
   * @brief Constructs a EndEffectorKinematics.
   */
  EndEffectorKinematics() = default;
  /**
   * @brief Destructor.
   */
  virtual ~EndEffectorKinematics() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual EndEffectorKinematics* Clone() const = 0;
  EndEffectorKinematics& operator=(const EndEffectorKinematics&) = delete;

  /**
   * @brief Get end-effector IDs (names).
   * @return Result value.
   */
  virtual const std::vector<std::string>& GetIds() const = 0;

  /**
   * Get end-effector position vectors in world frame
   *
   * @param [in] state: vector
   * @return array of position vectors
   */
  virtual std::vector<vector3_t> GetPosition(const vector_t& state) const = 0;

  /**
   * Get end-effector velocity vectors in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity vectors
   */
  virtual std::vector<vector3_t> GetVelocity(const vector_t& state, const vector_t& input) const = 0;

  /**
   * @brief Get orientation error in world frame.
   * @note: To calculate the error use QuaternionDistance() from autoocp/tools/common/rotation_transforms.hpp
   * @param [in] state: vector
   * @param [in] referenceOrientations: Reference Orientations.
   * @return array of orientation errors
   */
  virtual std::vector<vector3_t> GetOrientationError(const vector_t& state,
                                                     const std::vector<quaternion_t>& referenceOrientations) const = 0;

  /**
   * Get end-effector position linear approximation in world frame
   *
   * @param [in] state: state vector
   * @return array of position function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> GetPositionLinearApproximation(const vector_t& state) const = 0;

  /**
   * Get end-effector velocity linear approximation in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> GetVelocityLinearApproximation(const vector_t& state,
                                                                                        const vector_t& input) const = 0;

  /**
   * @brief Get end-effector orintation error linear approximation in world frame.
   *
   * autoocp/tools/common/rotation_transforms.hpp
   * @note: To calculate the error and Jacobian use QuaternionDistance() and QuaternionDistanceJacobian() from
   * @param [in] state: state vector
   * @param [in] referenceOrientations: Reference Orientations.
   * @return array of orientation error linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> GetOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const = 0;

 protected:
  /**
   * @brief Constructs a EndEffectorKinematics.
   */
  EndEffectorKinematics(const EndEffectorKinematics&) = default;
};

}  // namespace bolza
