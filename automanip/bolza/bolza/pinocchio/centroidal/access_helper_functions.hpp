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
 * @file access_helper_functions.hpp
 * @brief Provides read/write access to the contact forces.
 */


#include <Eigen/Core>

#include "bolza/pinocchio/centroidal/centroidal_model_info.hpp"

namespace bolza {
namespace centroidal_model {

/**
 * Provides read/write access to the contact forces.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Forces.
 * @param [in,out] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 3, 1> GetContactForces(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                             const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the contact forces.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Forces.
 * @param [in] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 3, 1> GetContactForces(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                         const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the contact torques.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Torques.
 * @param [in,out] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 3, 1> GetContactTorques(Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                              const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the contact torques.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Contact Torques.
 * @param [in] input: Input.
 * @param [in] contactIndex: Contact Index.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 3, 1> GetContactTorques(const Eigen::MatrixBase<Derived>& input, size_t contactIndex,
                                                          const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the joint velocities.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Velocities.
 * @param [in,out] input: Input.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, -1, 1> GetJointVelocities(Eigen::MatrixBase<Derived>& input, const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the joint velocities.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Velocities.
 * @param [in] input: Input.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, -1, 1> GetJointVelocities(const Eigen::MatrixBase<Derived>& input,
                                                            const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the mass-normalized momentum.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Normalized Momentum.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 6, 1> GetNormalizedMomentum(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the mass-normalized momentum.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Normalized Momentum.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 6, 1> GetNormalizedMomentum(const Eigen::MatrixBase<Derived>& state,
                                                              const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the base pose.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Base Pose.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, 6, 1> GetBasePose(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the base pose.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Base Pose.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, 6, 1> GetBasePose(const Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the joint angles.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Angles.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, -1, 1> getJointAngles(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the joint angles.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Joint Angles.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, -1, 1> getJointAngles(const Eigen::MatrixBase<Derived>& state,
                                                        const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read/write access to the generalized coordinates.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Generalized Coordinates.
 * @param [in,out] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
Eigen::Block<Derived, -1, 1> getGeneralizedCoordinates(Eigen::MatrixBase<Derived>& state, const CentroidalModelInfoTpl<SCALAR>& info);

/**
 * Provides read access to the generalized coordinates.
 */
template <typename Derived, typename SCALAR>
/**
 * @brief Get Generalized Coordinates.
 * @param [in] state: State.
 * @param [in] info: Info.
 * @return Result value.
 */
const Eigen::Block<const Derived, -1, 1> getGeneralizedCoordinates(const Eigen::MatrixBase<Derived>& state,
                                                                   const CentroidalModelInfoTpl<SCALAR>& info);

}  // namespace centroidal_model
}  // namespace bolza

#include "implementation/access_helper_functions_impl.hpp"
