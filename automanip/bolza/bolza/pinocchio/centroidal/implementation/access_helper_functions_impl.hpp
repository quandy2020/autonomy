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

namespace bolza {
namespace centroidal_model {

/******************************************************************************************************/

/**
 * @file access_helper_functions_impl.hpp
 * @brief access helper functions impl (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Forces.
 * @param [in,out] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 3, 1> GetContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                             const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  const size_t contactForceIndex = 3 * contactIndex;
  const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
  const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  /**
   * @brief Derived.
   * @param [in] startRow: Start Row.
   * @return Result value.
   */
  return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Forces.
 * @param [in] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 3, 1> GetContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                         const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  const size_t contactForceIndex = 3 * contactIndex;
  const size_t contactWrenchIndex = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts);
  const size_t startRow = (contactIndex < info.numThreeDofContacts) ? contactForceIndex : contactWrenchIndex;
  /**
   * @brief Derived.
   * @param [in] startRow: Start Row.
   * @return Result value.
   */
  return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Torques.
 * @param [in,out] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 3, 1> GetContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                              const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  assert(contactIndex >= info.numThreeDofContacts);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
  /**
   * @brief Derived.
   * @param [in] startRow: Start Row.
   * @return Result value.
   */
  return Eigen::Block<Derived, 3, 1>(input.derived(), startRow, 0);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Torques.
 * @param [in] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 3, 1> GetContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                          const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  assert(contactIndex < info.numThreeDofContacts + info.numSixDofContacts);
  assert(contactIndex >= info.numThreeDofContacts);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactIndex - info.numThreeDofContacts) + 3;
  /**
   * @brief Derived.
   * @param [in] startRow: Start Row.
   * @return Result value.
   */
  return Eigen::Block<const Derived, 3, 1>(input.derived(), startRow, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Velocities.
 * @param [in,out] input: Input.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, -1, 1> GetJointVelocities(Eigen::MatrixBase<Derived>& input, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  /**
   * @brief Derived.
   * @param [in] startRow: Start Row.
   * @param [in] actuatedDofNum: Actuated Dof Num.
   * @return Result value.
   */
  return Eigen::Block<Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Velocities.
 * @param [in] input: Input.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, -1, 1> GetJointVelocities(const Eigen::MatrixBase<Derived>& input,
                                                            const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(input.rows() == info.inputDim);
  assert(input.cols() == 1);
  const size_t startRow = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  /**
   * @brief Derived.
   * @param [in] startRow: Start Row.
   * @param [in] actuatedDofNum: Actuated Dof Num.
   * @return Result value.
   */
  return Eigen::Block<const Derived, -1, 1>(input.derived(), startRow, 0, info.actuatedDofNum, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Normalized Momentum.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 6, 1> GetNormalizedMomentum(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @return Result value.
   */
  return Eigen::Block<Derived, 6, 1>(state.derived(), 0, 0);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Normalized Momentum.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 6, 1> GetNormalizedMomentum(const Eigen::MatrixBase<Derived>& state,
                                                              const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @return Result value.
   */
  return Eigen::Block<const Derived, 6, 1>(state.derived(), 0, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Base Pose.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 6, 1> GetBasePose(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @return Result value.
   */
  return Eigen::Block<Derived, 6, 1>(state.derived(), 6, 0);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Base Pose.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 6, 1> GetBasePose(const Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @return Result value.
   */
  return Eigen::Block<const Derived, 6, 1>(state.derived(), 6, 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Angles.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @param [in] actuatedDofNum: Actuated Dof Num.
   * @return Result value.
   */
  return Eigen::Block<Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Angles.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
                                                        const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @param [in] actuatedDofNum: Actuated Dof Num.
   * @return Result value.
   */
  return Eigen::Block<const Derived, -1, 1>(state.derived(), 12, 0, info.actuatedDofNum, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived, typename SCALAR>
/**
 * @brief Get Generalized Coordinates.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @param [in] generalizedCoordinatesNum: Generalized Coordinates Num.
   * @return Result value.
   */
  return Eigen::Block<Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
}

template <typename Derived, typename SCALAR>
/**
 * @brief Get Generalized Coordinates.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
                                                                   const CentroidalModelInfoTpl<SCALAR>& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  /**
   * @brief Derived.
   * @param [in] generalizedCoordinatesNum: Generalized Coordinates Num.
   * @return Result value.
   */
  return Eigen::Block<const Derived, -1, 1>(state.derived(), 6, 0, info.generalizedCoordinatesNum, 1);
}

}  // namespace centroidal_model
}  // namespace bolza
