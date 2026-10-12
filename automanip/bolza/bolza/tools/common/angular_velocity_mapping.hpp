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
 * @file angular_velocity_mapping.hpp
 * @brief Compute the mapping from angular velocity w to rotation quaternion time derivative q_dot.
 */


#include <Eigen/Core>
#include <array>
#include <cmath>

namespace bolza {

/**
 * Compute the mapping from angular velocity w to rotation quaternion time derivative q_dot.
 *
 * @param [in]: q: orientation quaternion
 * @return mapping matrix E_R inverse, mapping angular velocity to quaternion time derivative.
 *         The colums are in the order [x, y, z, w] (default Eigen order as in q.coeffs()).
 */
template <typename SCALAR_T>
/**
 * @brief Angular Velocity To Quaternion Time Derivative.
 * @param [in] q: Q.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 4, 3> AngularVelocityToQuaternionTimeDerivative(const Eigen::Quaternion<SCALAR_T>& q) {
  const SCALAR_T qx = SCALAR_T(0.5) * q.x();
  const SCALAR_T qy = SCALAR_T(0.5) * q.y();
  const SCALAR_T qz = SCALAR_T(0.5) * q.z();
  const SCALAR_T qw = SCALAR_T(0.5) * q.w();
  Eigen::Matrix<SCALAR_T, 4, 3> M;
  // clang-format off
  // Robot Dynamics 2018, equation (2.97)
  M <<  qw,  qz, -qy,
       -qz,  qw,  qx,
        qy, -qx,  qw,
       -qx, -qy, -qz;
  // clang-format on
  return M;
}

/**
 * @brief Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
 *
 * @param [in] eulerAngles: euler angles in XYZ convention
 * @return M: matrix that does the transformation
 */
template <typename SCALAR_T>
/**
 * @brief Get Mapping From Local Angular Velocity To Euler Angles Xyz Derivative.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> GetMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T sz = sin(eulerAngles(2));
  const SCALAR_T cz = cos(eulerAngles(2));
  const SCALAR_T cz_cy = cz / cy;
  const SCALAR_T sz_cy = sz / cy;

  Eigen::Matrix<SCALAR_T, 3, 3> M;
  // clang-format off
  /**
   * @brief SCALAR T.
   * @param [in] sz: Sz.
   * @param [in] cz: Cz.
   * @param [in,out] cz_cy: Cz cy.
   * @param [in,out] sz_cy: Sz cy.
   * @return Result value.
   */
  M <<      cz_cy,       -sz_cy,   SCALAR_T(0.0),
               sz,           cz,   SCALAR_T(0.0),
      -sy * cz_cy,   sy * sz_cy,   SCALAR_T(1.0);
  // clang-format on

  return M;
}

/**
 * @brief Jacobian of mapping from local angular velocity to Euler-angle rates (auto-diff friendly).
 *
 * Maps body-frame angular velocity \omega_W to Euler-angle rates via \dot{q}_I = H \omega_W.
 * Returns stacked [dH/dq_I \omega_W; H].
 *
 * @param [in] eulerAnglesXyz: Euler XYZ angles.
 * @param [in] angularVelocity: Angular velocity in body coordinates.
 * @return 6x3 Jacobian matrix.
 */
template <typename SCALAR_T>
inline Eigen::Matrix<SCALAR_T, 6, 3> JacobianOfAngularVelocityMapping(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXyz,
                                                                      const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  using std::cos;
  using std::sin;

  Eigen::Matrix<SCALAR_T, 6, 1> x;
  x << eulerAnglesXyz, angularVelocity;

  std::array<SCALAR_T, 10> v;

  Eigen::Matrix<SCALAR_T, 6, 3> jac;
  /**
   * @brief Y.
   * @return Result value.
   */
  Eigen::Map<Eigen::Matrix<SCALAR_T, 3 * 6, 1>> y(jac.data());

#pragma clang diagnostic push
#pragma clang diagnostic ignored "cppcoreguidelines-pro-bounds-pointer-arithmetic"
  y[9] = sin(x[2]);
  y[10] = cos(x[2]);
  v[0] = cos(x[1]);
  v[1] = 1 / v[0];
  y[3] = v[1] * y[10];
  v[2] = sin(x[1]);
  y[1] = 0 - (0 - (0 - x[4] * y[9] + x[3] * y[10]) * 1 / v[0] * v[1]) * v[2];
  v[3] = sin(x[2]);
  v[4] = 0 - v[1];
  y[4] = v[4] * y[9];
  v[5] = y[10];
  y[2] = 0 - x[3] * v[1] * v[3] + x[4] * v[4] * v[5];
  y[8] = 0 - x[4] * v[3] + x[3] * v[5];
  v[6] = v[1] * y[9];
  v[7] = v[4] * y[10];
  v[8] = v[2];
  y[15] = v[7] * v[8];
  y[16] = v[6] * v[8];
  v[9] = x[4] * v[8];
  v[8] = x[3] * v[8];
  y[13] = (x[4] * v[6] + x[3] * v[7]) * v[0] - (0 - (v[9] * y[9] - v[8] * y[10]) * 1 / v[0] * v[1]) * v[2];
  y[14] = 0 - v[8] * v[4] * v[3] + v[9] * v[1] * v[5];
  // dependent variables without operations
  y[0] = 0;
  y[5] = 0;
  y[6] = 0;
  y[7] = 0;
  y[11] = 0;
  y[12] = 0;
  y[17] = 1;
#pragma clang diagnostic pop

  return jac;
}

}  // namespace bolza
