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
 * @file model_helper_functions.hpp
 * @brief Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 */


#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include "bolza/pinocchio/centroidal/centroidal_model_info.hpp"

namespace bolza {

/**
 * Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 *  Ab_inv = [  1/m I_{3,3},    -1/m*Ab_12*Ab_22^(-1),
 *              O_{3,3},         Ab_22^(-1)          ]
 *
 * @param [in] A(q): centroidal momentum matrix
 * @return Ab_inv(q): inverse of the 6x6 left-block of A(q)
 */
template <typename SCALAR_T>
/**
 * @brief Compute Floating Base Centroidal Momentum Matrix Inverse.
 * @param [in] Ab: Ab.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 6, 6> ComputeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab);

/**
 * Updates the centroidal momentum matrix in data.Ag and the CoM position in data.com[0] for the FullCentroidalDynamics
 * model and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 *
 * @remark: This function also internally calls:
 *       pinocchio::forwardKinematics(model, data, q)
 *       pinocchio::computeJointJacobians(model, data, q) (only for the FullCentroidalDynamics case)
 *       pinocchio::updateFramePlacements(model, data)
 */
template <typename SCALAR_T>
/**
 * @brief Update Centroidal Dynamics.
 * @param [in,out] interface: Interface.
 * @param [in] info: Info.
 * @param [in] q: Q.
 */
void UpdateCentroidalDynamics(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                              const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q);

/**
 * Updates the centroidal momentum derivatives (such as in data.dHdq) for the FullCentroidalDynamics model
 * and the SingleRigidBodyDynamics Model
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
 *
 * @remark: This function also internally calls:
 *       pinocchio::forwardKinematics(model, data, q)
 *       pinocchio::computeJointJacobians(model, data, q)
 *       pinocchio::updateFramePlacements(model, data)
 */
template <typename SCALAR_T>
/**
 * @brief Update Centroidal Dynamics Derivatives.
 * @param [in,out] interface: Interface.
 * @param [in] info: Info.
 * @param [in] q: Q.
 * @param [in] v: V.
 */
void UpdateCentroidalDynamicsDerivatives(PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                         const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v);

/**
 * Computes derivatives of the mapping (ZYX-Euler angles derivatives --> Global angular velocities)
 * with respect to the base orientation (ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the mapping w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
/**
 * @brief Get Mapping Zyx Gradient.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> GetMappingZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

/**
 * Computes derivatives of the rotation Matrix (base frame --> world frame) with respect to
 * the base orientation (in ZYX-Euler angles)
 *
 * @param [in] eulerAngles: ZYX-Euler angles extracted from qPinocchio
 * @return A tensor representing the derivative of the rotation matrix w.r.t the ZYX-Euler angles
 */
template <typename SCALAR_T>
/**
 * @brief Get Rotation Matrix Zyx Gradient.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
std::array<Eigen::Matrix<SCALAR_T, 3, 3>, 3> GetRotationMatrixZyxGradient(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles);

/**
 * Computes derivatives of centroidal momentum with respect to the base orientation (in ZYX-Euler angles)
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] q: pinocchio joint positions (generalized coordinates)
 * @param [in] v: pinocchio joint velocities (derivatives of generalized coordinates)
 * @return Derivative of centroidal momentum w.r.t the ZYX-Euler Angles
 */
template <typename SCALAR_T>
/**
 * @brief Get Centroidal Momentum Zyx Gradient.
 * @param [in] interface: Interface.
 * @param [in] info: Info.
 * @param [in] q: Q.
 * @param [in] v: V.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 6, 3> GetCentroidalMomentumZyxGradient(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                               const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& q,
                                                               const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& v);

/**
 * Returns the centroidal momentum matrix from the pinocchioInterface data
 * @param [in] interface: pinocchio robot interface containing model + data
 * @return centroidal momentum matrix from data.Ag
 *
 * @note requires pinocchioInterface to be updated with:
 *       bolza::UpdateCentroidalDynamics(interface, info, q)
 */
template <typename SCALAR_T>
/**
 * @brief Get Centroidal Momentum Matrix.
 * @param [in] interface: Interface.
 * @return Result value.
 */
const Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>& GetCentroidalMomentumMatrix(const PinocchioInterfaceTpl<SCALAR_T>& interface);

/**
 * Computes the CoM to contact point position in world frame
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] contactIndex: index of the contact point
 * @return position of the contact point w.r.t CoM expressed in world frame
 *
 * @note requires pinocchioInterface to be updated with:
 *       bolza::UpdateCentroidalDynamics(interface, info, q)
 */
template <typename SCALAR_T>
/**
 * @brief Get Position Com To Contact Point In World Frame.
 * @param [in] interface: Interface.
 * @param [in] info: Info.
 * @param [in] contactIndex: Contact Index.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> GetPositionComToContactPointInWorldFrame(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                       const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex);

/**
 * Computes the CoM to contact point translational Jacobian in world frame
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] contactIndex: index of the contact point
 * @return CoM to contact point translational Jacobian expressed in world frame
 *
 * @note requires pinocchioInterface to be updated with:
 *       bolza::UpdateCentroidalDynamics(interface, info, q) (should be called first)
 *       pinocchio::computeJointJacobians(model, data, q)
 *       pinocchio::updateFramePlacements(model, data)
 */
// TODO: Need to copy data here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
template <typename SCALAR_T>
/**
 * @brief Get Translational Jacobian Com To Contact Point In World Frame.
 * @param [in] interface: Interface.
 * @param [in] info: Info.
 * @param [in] contactIndex: Contact Index.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic> GetTranslationalJacobianComToContactPointInWorldFrame(
    const PinocchioInterfaceTpl<SCALAR_T>& interface, const CentroidalModelInfoTpl<SCALAR_T>& info, size_t contactIndex);

/**
 * Computes the derivative of the normalized centroidal momentum (linear + angular) expressed in the centroidal frame
 *
 * @param [in] interface: pinocchio robot interface containing model + data
 * @param [in] info: centroidal model information
 * @param [in] input: system input vector
 * @return time derivative of normalized centroidal momentum
 *
 * @note requires pinocchioInterface to be updated with:
 *       bolza::UpdateCentroidalDynamics(interface, info, q)
 */
template <typename SCALAR_T>
/**
 * @brief Get Normalized Centroidal Momentum Rate.
 * @param [in] interface: Interface.
 * @param [in] info: Info.
 * @param [in] input: Input.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 6, 1> GetNormalizedCentroidalMomentumRate(const PinocchioInterfaceTpl<SCALAR_T>& interface,
                                                                  const CentroidalModelInfoTpl<SCALAR_T>& info,
                                                                  const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& input);

}  // namespace bolza

#include "implementation/model_helper_functions_impl.hpp"
