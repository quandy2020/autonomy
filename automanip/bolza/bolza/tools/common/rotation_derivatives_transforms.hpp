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
 * @file rotation_derivatives_transforms.hpp
 * @brief Compute the matrix that maps derivatives of ZYX-Euler angles to global angular velocities.
 */


#include <Eigen/Core>
#include <array>
#include <cmath>

namespace bolza {

/**
 * Compute the matrix that maps derivatives of ZYX-Euler angles to global angular velocities
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @return 3x3 matrix mapping
 */
template <typename SCALAR_T>
/**
 * @brief Get Mapping From Euler Angles Zyx Derivative To Global Angular Velocity.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> GetMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T sz = sin(eulerAngles(0));
  const SCALAR_T cz = cos(eulerAngles(0));
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  transformationMatrix << SCALAR_T(0.0),            -sz,    cy*cz,
                          SCALAR_T(0.0),             cz,    cy*sz,
                          SCALAR_T(1.0),  SCALAR_T(0.0),      -sy;
  // clang-format on
  return transformationMatrix;
}

/**
 * Compute angular velocities expressed in the world frame from derivatives of ZYX-Euler angles
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] derivativesEulerAngles: time-derivative of ZYX-Euler angles
 * @return angular velocity expressed in world frame
 */
template <typename SCALAR_T>
/**
 * @brief Get Global Angular Velocity From Euler Angles Zyx Derivatives.
 * @param [in] eulerAngles: Euler Angles.
 * @param [in] derivativesEulerAngles: Derivatives Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> GetGlobalAngularVelocityFromEulerAnglesZyxDerivatives(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles) {
  const SCALAR_T sz = sin(eulerAngles(0));
  const SCALAR_T cz = cos(eulerAngles(0));
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T dz = derivativesEulerAngles(0);
  const SCALAR_T dy = derivativesEulerAngles(1);
  const SCALAR_T dx = derivativesEulerAngles(2);
  return {-sz * dy + cy * cz * dx, cz * dy + cy * sz * dx, dz - sy * dx};
}

/**
 * Compute derivatives of ZYX-Euler angles from global angular velocities
 * The inverse of getGlobalAngularVelocityFromEulerAnglesZyxDerivatives
 *
 * This transformation is singular for y = +- pi / 2
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] angularVelocity: angular velocity expressed in world frame
 * @return derivatives of ZYX-Euler angles
 */
template <typename SCALAR_T>
/**
 * @brief Get Euler Angles Zyx Derivatives From Global Angular Velocity.
 * @param [in] eulerAngles: Euler Angles.
 * @param [in] angularVelocity: Angular Velocity.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> GetEulerAnglesZyxDerivativesFromGlobalAngularVelocity(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                                                    const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  const SCALAR_T sz = sin(eulerAngles(0));
  const SCALAR_T cz = cos(eulerAngles(0));
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T wx = angularVelocity(0);
  const SCALAR_T wy = angularVelocity(1);
  const SCALAR_T wz = angularVelocity(2);
  const SCALAR_T tmp = cz * wx / cy + sz * wy / cy;
  return {sy * tmp + wz, -sz * wx + cz * wy, tmp};
}

/**
 * Compute angular accelerations expressed in the world frame from derivatives of ZYX-Euler angles
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] derivativesEulerAngles: time-derivative of ZYX-Euler angles
 * @param [in] secondDerivativesEulerAngles: second order time-derivative of ZYX-Euler angles
 * @return angular acceleration expressed in world frame
 */
template <typename SCALAR_T>
/**
 * @brief Get Global Angular Acceleration From Euler Angles Zyx Derivatives.
 * @param [in] eulerAngles: Euler Angles.
 * @param [in] derivativesEulerAngles: Derivatives Euler Angles.
 * @param [in] secondDerivativesEulerAngles: Second Derivatives Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> GetGlobalAngularAccelerationFromEulerAnglesZyxDerivatives(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles,
    const Eigen::Matrix<SCALAR_T, 3, 1>& secondDerivativesEulerAngles) {
  const SCALAR_T sz = sin(eulerAngles(0));
  const SCALAR_T cz = cos(eulerAngles(0));
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T sx = sin(eulerAngles(2));
  const SCALAR_T cx = cos(eulerAngles(2));
  const SCALAR_T dz = derivativesEulerAngles(0);
  const SCALAR_T dy = derivativesEulerAngles(1);
  const SCALAR_T dx = derivativesEulerAngles(2);
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix = GetMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  Eigen::Matrix<SCALAR_T, 3, 3> derivativeTransformationMatrix;
  // clang-format off
  derivativeTransformationMatrix << SCALAR_T(0), -cz*dz, (-sy*dy)*cz + cy*(-sz*dz),
                                    SCALAR_T(0), -sz*dz,  (-sy*dy)*sz + cy*(cz*dz),
                                    SCALAR_T(0),  SCALAR_T(0),   -cy*dy;
  // clang-format on
  return derivativeTransformationMatrix * derivativesEulerAngles + transformationMatrix * secondDerivativesEulerAngles;
}

/**
 * Compute the matrix that maps derivatives of ZYX-Euler angles to local angular velocities
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @return 3x3 matrix mapping
 */
template <typename SCALAR_T>
/**
 * @brief Get Mapping From Euler Angles Zyx Derivative To Local Angular Velocity.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> GetMappingFromEulerAnglesZyxDerivativeToLocalAngularVelocity(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T sx = sin(eulerAngles(2));
  const SCALAR_T cx = cos(eulerAngles(2));
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  /**
   * @brief SCALAR T.
   * @param [in,out] sx: Sx.
   * @param [in] cx: Cx.
   * @param [in,out] cy: Cy.
   * @param [in] sx: Sx.
   * @return Result value.
   */
  transformationMatrix <<     -sy,  SCALAR_T(0.0),    SCALAR_T(1.0),
                          cy * sx,             cx,    SCALAR_T(0.0),
                          cx * cy,            -sx,    SCALAR_T(0.0);
  // clang-format on
  return transformationMatrix;
}

/**
 * Compute angular velocities expressed in the local frame from derivatives of ZYX-Euler angles
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] derivativesEulerAngles: time-derivative of ZYX-Euler angles
 * @return angular velocity expressed in local frame
 */
template <typename SCALAR_T>
/**
 * @brief Get Local Angular Velocity From Euler Angles Zyx Derivatives.
 * @param [in] eulerAngles: Euler Angles.
 * @param [in] derivativesEulerAngles: Derivatives Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> GetLocalAngularVelocityFromEulerAnglesZyxDerivatives(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles) {
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T sx = sin(eulerAngles(2));
  const SCALAR_T cx = cos(eulerAngles(2));
  const SCALAR_T dz = derivativesEulerAngles(0);
  const SCALAR_T dy = derivativesEulerAngles(1);
  const SCALAR_T dx = derivativesEulerAngles(2);
  return {-sy * dz + dx, cy * sx * dz + cx * dy, cx * cy * dz - sx * dy};
}

/**
 * Compute derivatives of ZYX-Euler angles from local angular velocities
 * The inverse of getLocalAngularVelocityFromEulerAnglesZyxDerivatives
 *
 * This transformation is singular for y = +- pi / 2
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] angularVelocity: angular velocity expressed in local frame
 * @return derivatives of ZYX-Euler angles
 */
template <typename SCALAR_T>
/**
 * @brief Get Euler Angles Zyx Derivatives From Local Angular Velocity.
 * @param [in] eulerAngles: Euler Angles.
 * @param [in] angularVelocity: Angular Velocity.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> GetEulerAnglesZyxDerivativesFromLocalAngularVelocity(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                                                   const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T sx = sin(eulerAngles(2));
  const SCALAR_T cx = cos(eulerAngles(2));
  const SCALAR_T wx = angularVelocity(0);
  const SCALAR_T wy = angularVelocity(1);
  const SCALAR_T wz = angularVelocity(2);
  const SCALAR_T tmp = sx * wy / cy + cx * wz / cy;
  return {tmp, cx * wy - sx * wz, wx + sy * tmp};
}

}  // namespace bolza