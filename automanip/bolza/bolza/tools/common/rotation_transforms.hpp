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
 * @file rotation_transforms.hpp
 * @brief Compute the quaternion distance measure.
 */


#include <Eigen/Core>
#include <array>
#include <cmath>

// CppAD
#include <bolza/core/automatic_differentiation/types.hpp>

namespace bolza {

/**
 * Compute the quaternion distance measure
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x1 vector representing the quaternion distance.
 * In particular, if Qd and Qe are the desired and the measured end-effector
 * quaternions, the measured and desired frames are aligned if this vector is 0.
 */
template <typename SCALAR_T>
/**
 * @brief Quaternion Distance.
 * @param [in] q: Q.
 * @param [in] qRef: Q Ref.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> QuaternionDistance(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  /**
   * @brief W.
   * @return Result value.
   */
  return q.w() * qRef.vec() - qRef.w() * q.vec() + q.vec().cross(qRef.vec());
}

/**
 * Compute the quaternion distance measure jacobian
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x4 matrix representing the quaternion distance jacobian.
 * The columns are the partial derivatives of [q.x, q.y, q,z, qw] (default Eigen order)
 */
template <typename SCALAR_T>
/**
 * @brief Quaternion Distance Jacobian.
 * @param [in] q: Q.
 * @param [in] qRef: Q Ref.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 4> QuaternionDistanceJacobian(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  Eigen::Matrix<SCALAR_T, 3, 4> jacobian;
  // clang-format off
  /**
   * @brief W.
   * @return Result value.
   */
  jacobian << -qRef.w(), qRef.z(), -qRef.y(), qRef.x(),
              -qRef.z(), -qRef.w(), qRef.x(), qRef.y(),
              qRef.y(), -qRef.x(), -qRef.w(), qRef.z();
  // clang-format on
  return jacobian;
}

/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
/**
 * @brief Get Quaternion From Euler Angles Zyx.
 * @param [in] eulerAnglesZyx: Euler Angles Zyx.
 * @return Result value.
 */
Eigen::Quaternion<SCALAR_T> GetQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  // clang-format off
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
         /**
          * @brief Euler Angles Zyx.
          * @return Result value.
          */
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX());
  // clang-format on
}

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename SCALAR_T>
/**
 * @brief Get Rotation Matrix From Zyx Euler Angles.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> GetRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
  // clang-format on
  return rotationMatrix;
}

/**
 * @brief Transform a set of Euler Angles (each in [-pi, pi)) into Euler Angles in the range [-pi,pi),[-pi/2,pi/2),[-pi,pi)
 * @param [in,out] Reference: to eulerAngles XYZ or ZYX which will be modified in place
 * @note Code taken from https://github.com/ANYbotics/kindr/blob/master/include/kindr/rotations/EulerAnglesXyz.hpp
 * @note Works for Euler Angles XYZ and ZYX alike
 */
template <typename SCALAR_T>
/**
 * @brief Make Euler Angles Unique.
 * @param [in,out] eulerAngles: Euler Angles.
 */
void MakeEulerAnglesUnique(Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  /**
   * @brief Tol.
   * @return Result value.
   */
  SCALAR_T tol(1e-9);  // FIXME(jcarius) magic number
  /**
   * @brief Pi.
   * @param [in] M_PI: M PI.
   * @return Result value.
   */
  SCALAR_T pi(M_PI);

  if (eulerAngles.y() < -pi / 2 - tol) {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    /**
     * @brief Y.
     * @return Result value.
     */
    eulerAngles.y() = -(eulerAngles.y() + pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  } else if (-pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= -pi / 2 + tol) {
    /**
     * @brief X.
     * @return Result value.
     */
    eulerAngles.x() -= eulerAngles.z();
    /**
     * @brief Z.
     * @return Result value.
     */
    eulerAngles.z() = 0;
  } else if (-pi / 2 + tol < eulerAngles.y() && eulerAngles.y() < pi / 2 - tol) {
    // ok
  } else if (pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= pi / 2 + tol) {
    // todo: pi/2 should not be in range, other formula?
    /**
     * @brief X.
     * @return Result value.
     */
    eulerAngles.x() += eulerAngles.z();
    /**
     * @brief Z.
     * @return Result value.
     */
    eulerAngles.z() = 0;
  } else  // pi/2 + Tol < eulerAngles.y()
  {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    /**
     * @brief Y.
     * @return Result value.
     */
    eulerAngles.y() = -(eulerAngles.y() - pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  }
}

/**
 * Compute a quaternion from a matrix
 *
 * @param [in] R: Rotation Matrix.
 * @return A quaternion representing an equivalent rotation to R.
 */
template <typename SCALAR_T>
/**
 * @brief Matrix To Quaternion.
 * @param [in] R: R.
 * @return Result value.
 */
Eigen::Quaternion<SCALAR_T> MatrixToQuaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
  return Eigen::Quaternion<SCALAR_T>(R);
}

/**
 * Compute a quaternion from a matrix, specialized for CppAd (the default Eigen implementation uses branches)
 *
 * @param [in] R: Rotation Matrix templated on ad_scalar_t.
 * @return A quaternion representing an equivalent rotation to R.
 */
Eigen::Quaternion<ad_scalar_t> MatrixToQuaternion(const Eigen::Matrix<ad_scalar_t, 3, 3>& R);

/**
 * Returns the logarithmic map of the rotation
 *      w = theta * n = log(R);
 *
 * Will find an angle, theta, in the interval [0, pi]
 *
 * To make the computation numerically stable and compatible with autodiff, we have to switch between 3 cases.
 * - For most angles we use the logarithmic map directly
 * - For angles close to 0.0, we use the taylor expansion
 * - For angles close to PI, we use a quaternion based solution.
 *
 * @tparam SCALAR_T : numeric type
 * @param [in] rotationMatrix: 3x3 rotation matrix
 * @return 3x1 rotation vector, theta * n, with theta equal to the rotation angle, and n equal to the rotation axis.
 */
template <typename SCALAR_T>
/**
 * @brief Rotation Matrix To Rotation Vector.
 * @param [in] rotationMatrix: Rotation Matrix.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotationMatrixToRotationVector(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrix) {
  // Helper function to select a 3d vector compatible with CppAd
  auto selectSolutionGt = [](SCALAR_T left, SCALAR_T right, const Eigen::Matrix<SCALAR_T, 3, 1>& if_true,
                             const Eigen::Matrix<SCALAR_T, 3, 1>& if_false) -> Eigen::Matrix<SCALAR_T, 3, 1> {
    return {CppAD::CondExpGt(left, right, if_true[0], if_false[0]), CppAD::CondExpGt(left, right, if_true[1], if_false[1]),
            CppAD::CondExpGt(left, right, if_true[2], if_false[2])};
  };
  const auto& R = rotationMatrix;

  const SCALAR_T trace = R(0, 0) + R(1, 1) + R(2, 2);
  /**
   * @brief Skew Vector.
   * @return Result value.
   */
  const Eigen::Matrix<SCALAR_T, 3, 1> skewVector(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

  // Tolerance to select alternative solution near singularity
  /**
   * @brief Eps.
   * @return Result value.
   */
  const SCALAR_T eps(1e-8);
  const SCALAR_T smallAngleThreshold = SCALAR_T(3.0) - eps;   // select taylorExpansionSol if trace > 3 - eps
  const SCALAR_T largeAngleThreshold = -SCALAR_T(1.0) + eps;  // select quaternionSol if trace < -1.0 + eps

  // Clip trace away from singularities, to be used in branches that might result in NaN.
  const SCALAR_T safeHighTrace =
      /**
       * @brief Cond Exp Gt.
       * @param [in] trace: Trace.
       * @param [in] smallAngleThreshold: Small Angle Threshold.
       * @param [in] smallAngleThreshold: Small Angle Threshold.
       * @param [in] trace: Trace.
       * @return Result value.
       */
      CppAD::CondExpGt(trace, smallAngleThreshold, smallAngleThreshold, trace);  // this trace is lower than 3 - eps
  const SCALAR_T safeTrace = CppAD::CondExpGt(safeHighTrace, largeAngleThreshold, safeHighTrace,
                                              largeAngleThreshold);  // this trace is also higher than -1.0 + eps

  // Rotation close to zero -> use taylor expansion, use when trace > 3.0 - eps
  const Eigen::Matrix<SCALAR_T, 3, 1> taylorExpansionSol = (SCALAR_T(0.75) - trace / SCALAR_T(12.0)) * skewVector;

  // Normal rotation, use normal logarithmic map
  const SCALAR_T tmp = SCALAR_T(0.5) * (safeTrace - SCALAR_T(1.0));
  const SCALAR_T theta = acos(tmp);
  const Eigen::Matrix<SCALAR_T, 3, 1> normalSol = (SCALAR_T(0.5) * theta / sqrt(SCALAR_T(1.0) - tmp * tmp)) * skewVector;

  // Quaternion solution, when close to pi, use when trace < -1.0 + eps
  auto q = bolza::MatrixToQuaternion(R);

  // Correct sign to make qw positive
  /**
   * @brief Vec.
   * @return Result value.
   */
  q.vec() = selectSolutionGt(q.w(), SCALAR_T(0.0), q.vec(), -q.vec());
  /**
   * @brief W.
   * @return Result value.
   */
  q.w() = CppAD::CondExpGt(q.w(), SCALAR_T(0.0), q.w(), -q.w());

  // Norm of vector part of the quaternion. Compute from trace to avoid squaring element and losing precision
  const SCALAR_T qVecNorm = SCALAR_T(0.5) * sqrt(SCALAR_T(3.0) - safeHighTrace);

  Eigen::Matrix<SCALAR_T, 3, 1> quaternionSol = SCALAR_T(4.0) * atan(qVecNorm / (q.w() + SCALAR_T(1.0))) * q.vec() / qVecNorm;

  // Select solution
  /**
   * @brief Select Solution Gt.
   * @param [in] trace: Trace.
   * @param [in] largeAngleThreshold: Large Angle Threshold.
   * @param [in] quaternionSol: Quaternion Sol.
   * @return Result value.
   */
  return selectSolutionGt(trace, largeAngleThreshold, selectSolutionGt(trace, smallAngleThreshold, taylorExpansionSol, normalSol),
                          quaternionSol);
}

/**
 * Computes a rotation error as a 3D vector in world frame. This 3D vector is the angle * axis representation of the rotation error.
 * This operation is also known as "box-minus": error = lhs [-] rhs
 *
 * Example usage:
 *    rotationErrorInWorld = RotationErrorInWorld(rotationBaseMeasuredToWorld, rotationBaseReferenceToWorld)
 *
 * @tparam SCALAR_T : numerical type
 * @param [in] rotationMatrixLhs: rotation from lhs frame to world
 * @param [in] rotationMatrixRhs: rotation from rhs frame to world
 * @return error = lhs [-] rhs
 */
template <typename SCALAR_T>
/**
 * @brief Rotation Error In World.
 * @param [in] rotationMatrixLhs: Rotation Matrix Lhs.
 * @param [in] rotationMatrixRhs: Rotation Matrix Rhs.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotationErrorInWorld(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixLhs,
                                                   const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixRhs) {
  /* Note that this error (W_R_lhs * rhs_R_W) does not follow the usual concatination of rotations.
   * It follows from simplifying:
   *    errorInWorld = W_R_lhs * angleAxis(rhs_R_W * W_R_lhs)
   *                 = angleAxis(W_R_lhs * rhs_R_W * W_R_lhs * lhs_R_W)
   *                 = angleAxis(W_R_lhs * rhs_R_W)
   */
  const Eigen::Matrix<SCALAR_T, 3, 3> rotationErrorInWorld = rotationMatrixLhs * rotationMatrixRhs.transpose();
  /**
   * @brief Rotation Matrix To Rotation Vector.
   * @param [in] rotationErrorInWorld: Rotation Error In World.
   * @return Result value.
   */
  return RotationMatrixToRotationVector(rotationErrorInWorld);
}

/**
 * The returned rotation error is expressed in the local frame (lhs or rhs).
 * Because the rotation error rotates between the lhs and rhs frame, its representation in both frames is numerically identical.
 *
 * Example usage:
 *    rotationErrorInBase = RotationErrorInLocal(rotationBaseMeasuredToWorld, rotationBaseReferenceToWorld)
 *
 * with the note that rotationErrorInBaseMeasured = rotationErrorInBaseReference
 *
 * See rotationErrorInWorld for further explanation.
 * @tparam SCALAR_T : numerical type
 * @param [in] rotationMatrixLhs: rotation from lhs frame to world
 * @param [in] rotationMatrixRhs: rotation from rhs frame to world
 * @return error = lhs [-] rhs
 */
template <typename SCALAR_T>
/**
 * @brief Rotation Error In Local.
 * @param [in] rotationMatrixLhs: Rotation Matrix Lhs.
 * @param [in] rotationMatrixRhs: Rotation Matrix Rhs.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotationErrorInLocal(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixLhs,
                                                   const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixRhs) {
  const Eigen::Matrix<SCALAR_T, 3, 3> rotationErrorInLocal = rotationMatrixRhs.transpose() * rotationMatrixLhs;
  /**
   * @brief Rotation Matrix To Rotation Vector.
   * @param [in] rotationErrorInLocal: Rotation Error In Local.
   * @return Result value.
   */
  return RotationMatrixToRotationVector(rotationErrorInLocal);
}

/**
 * Find an angle that is closest to a reference angle.
 *
 * @param [in] x: The input angle.
 * @param [in] reference: The reference angle.
 * @return An angle (x + k*2*pi) with k such that the result is within [reference - pi, reference + pi].
 */
scalar_t ModuloAngleWithReference(scalar_t x, scalar_t reference);

}  // namespace bolza