//
// Created by rgrandia on 22.10.19.
//

#pragma once

/**
 * @file rotations.hpp
 * @brief Base to origin rotation matrix.
 */


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include <bolza/tools/common/rotation_transforms.hpp>

namespace switched_model {

/**
 * Base to origin rotation matrix
 * @tparam Derived
 * @param [in] eulerAngles
 * @return
 */
template <typename SCALAR_T>
/**
 * @brief Rotation Matrix Base To Origin.
 * @param [in] eulerAnglesXYZ: Euler Angles XYZ.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> RotationMatrixBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  // Generated code for:
  //  // inputs are the intrinsic rotation angles in RADIANTS
  //  SCALAR_T sinAlpha = sin(eulerAngles(0));
  //  SCALAR_T cosAlpha = cos(eulerAngles(0));
  //  SCALAR_T sinBeta = sin(eulerAngles(1));
  //  SCALAR_T cosBeta = cos(eulerAngles(1));
  //  SCALAR_T sinGamma = sin(eulerAngles(2));
  //  SCALAR_T cosGamma = cos(eulerAngles(2));
  //
  //  Eigen::Matrix<SCALAR_T, 3, 3> Rx, Ry, Rz;
  //  Rx << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), cosAlpha, -sinAlpha, SCALAR_T(0), sinAlpha, cosAlpha;
  //  Ry << cosBeta, SCALAR_T(0), sinBeta, SCALAR_T(0), SCALAR_T(1), SCALAR_T(0), -sinBeta, SCALAR_T(0), cosBeta;
  //  Rz << cosGamma, -sinGamma, SCALAR_T(0), sinGamma, cosGamma, SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);
  //
  //  return Rx * Ry * Rz;
  Eigen::Matrix<SCALAR_T, 3, 3> o_R_b;

  // auxiliary variables
  std::array<SCALAR_T, 8> v{};

  v[0] = cos(eulerAnglesXYZ[1]);
  v[1] = cos(eulerAnglesXYZ[2]);
  o_R_b(0) = v[0] * v[1];
  v[2] = sin(eulerAnglesXYZ[0]);
  v[3] = -v[2];
  /**
   * @brief O R b.
   */
  o_R_b(6) = sin(eulerAnglesXYZ[1]);
  v[4] = -o_R_b(6);
  v[5] = v[3] * v[4];
  v[6] = cos(eulerAnglesXYZ[0]);
  v[7] = sin(eulerAnglesXYZ[2]);
  o_R_b(1) = v[5] * v[1] + v[6] * v[7];
  v[4] = v[6] * v[4];
  o_R_b(2) = v[4] * v[1] + v[2] * v[7];
  v[7] = -v[7];
  o_R_b(3) = v[0] * v[7];
  o_R_b(4) = v[5] * v[7] + v[6] * v[1];
  o_R_b(5) = v[4] * v[7] + v[2] * v[1];
  o_R_b(7) = v[3] * v[0];
  o_R_b(8) = v[6] * v[0];
  return o_R_b;
}

template <typename SCALAR_T>
/**
 * @brief Rotation Matrix Origin To Base.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> RotationMatrixOriginToBase(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  /**
   * @brief Transpose.
   * @return Result value.
   */
  return RotationMatrixBaseToOrigin<SCALAR_T>(eulerAngles).transpose();
}

template <typename SCALAR_T>
/**
 * @brief Rotate In Place2d.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void RotateInPlace2d(Eigen::Matrix<SCALAR_T, 2, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx - s * vy;
  v.y() = s * vx + c * vy;
}

template <typename SCALAR_T>
/**
 * @brief Inv Rotate In Place2d.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void InvRotateInPlace2d(Eigen::Matrix<SCALAR_T, 2, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx + s * vy;
  v.y() = -s * vx + c * vy;
}

template <typename SCALAR_T>
/**
 * @brief Rotate In Place Z.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void RotateInPlaceZ(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx - s * vy;
  v.y() = s * vx + c * vy;
}

template <typename SCALAR_T>
/**
 * @brief Inv Rotate In Place Z.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void InvRotateInPlaceZ(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx + s * vy;
  v.y() = -s * vx + c * vy;
}

template <typename SCALAR_T>
/**
 * @brief Rotate In Place Y.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void RotateInPlaceY(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vz = v.z();
  v.x() = c * vx + s * vz;
  v.z() = -s * vx + c * vz;
}

template <typename SCALAR_T>
/**
 * @brief Inv Rotate In Place Y.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void InvRotateInPlaceY(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vz = v.z();
  v.x() = c * vx - s * vz;
  v.z() = s * vx + c * vz;
}

template <typename SCALAR_T>
/**
 * @brief Rotate In Place X.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void RotateInPlaceX(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vy = v.y();
  const SCALAR_T vz = v.z();
  v.y() = c * vy - s * vz;
  v.z() = s * vy + c * vz;
}

template <typename SCALAR_T>
/**
 * @brief Inv Rotate In Place X.
 * @param [in,out] v: V.
 * @param [in] angle: Angle.
 */
void InvRotateInPlaceX(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vy = v.y();
  const SCALAR_T vz = v.z();
  v.y() = c * vy + s * vz;
  v.z() = -s * vy + c * vz;
}

/**
 * Directly rotates a vector from base to origin.
 *
 * Uses 12 (3*4) multiplications, and 6 (3*2) additions.
 *
 * Constructing just the rotation matrix already takes 14 multiplication, and 4 additions. Without having rotated the vector.
 * Multiplication with the rotation matrix takes 9 multiplications and 6 additions.
 * Constructing the rotation matrix could be cheaper when doing >5 rotations with the same matrix
 */
template <typename SCALAR_T>
/**
 * @brief Rotate Vector Base To Origin.
 * @param [in] v: V.
 * @param [in] eulerAnglesXYZ: Euler Angles XYZ.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotateVectorBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& v,
                                                       const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  Eigen::Matrix<SCALAR_T, 3, 1> rotatedVector = v;
  /**
   * @brief Constructs a RotateInPlaceZ.
   * @param [in] rotatedVector: Rotated Vector.
   */
  RotateInPlaceZ(rotatedVector, eulerAnglesXYZ[2]);
  /**
   * @brief Constructs a RotateInPlaceY.
   * @param [in] rotatedVector: Rotated Vector.
   */
  RotateInPlaceY(rotatedVector, eulerAnglesXYZ[1]);
  /**
   * @brief Constructs a RotateInPlaceX.
   * @param [in] rotatedVector: Rotated Vector.
   */
  RotateInPlaceX(rotatedVector, eulerAnglesXYZ[0]);
  return rotatedVector;
}

/**
 * inverse of rotateVectorBaseToOrigin, use here the invRotate helpers instead of rotate(-angle),
 * otherwise the sin/cos(angle) and sin/cos(-angle) cannot be merged by CppAd
 */
template <typename SCALAR_T>
/**
 * @brief Rotate Vector Origin To Base.
 * @param [in] v: V.
 * @param [in] eulerAnglesXYZ: Euler Angles XYZ.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotateVectorOriginToBase(const Eigen::Matrix<SCALAR_T, 3, 1>& v,
                                                       const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  Eigen::Matrix<SCALAR_T, 3, 1> rotatedVector = v;
  /**
   * @brief Constructs a InvRotateInPlaceX.
   * @param [in] rotatedVector: Rotated Vector.
   */
  InvRotateInPlaceX(rotatedVector, eulerAnglesXYZ[0]);
  /**
   * @brief Constructs a InvRotateInPlaceY.
   * @param [in] rotatedVector: Rotated Vector.
   */
  InvRotateInPlaceY(rotatedVector, eulerAnglesXYZ[1]);
  /**
   * @brief Constructs a InvRotateInPlaceZ.
   * @param [in] rotatedVector: Rotated Vector.
   */
  InvRotateInPlaceZ(rotatedVector, eulerAnglesXYZ[2]);
  return rotatedVector;
}

template <typename SCALAR_T>
/**
 * @brief Quaternion Base To Origin.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Quaternion<SCALAR_T> QuaternionBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  /**
   * @brief Euler Angles.
   * @return Result value.
   */
  return Eigen::Quaternion<SCALAR_T>(Eigen::AngleAxis<SCALAR_T>(eulerAngles(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
                                     Eigen::AngleAxis<SCALAR_T>(eulerAngles(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
                                     Eigen::AngleAxis<SCALAR_T>(eulerAngles(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()));
}

template <typename SCALAR_T>
/**
 * @brief Euler Angles From Quaternion Base To Origin.
 * @param [in] q_origin_base: Q origin base.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> EulerAnglesFromQuaternionBaseToOrigin(const Eigen::Quaternion<SCALAR_T>& q_origin_base) {
  /**
   * @brief To Rotation Matrix.
   * @return Result value.
   */
  return q_origin_base.toRotationMatrix().eulerAngles(0, 1, 2);
}

/**
 * Calculates the skew matrix for vector cross product
 * @tparam Derived
 * @param [in] in
 * @return
 */
template <typename SCALAR_T, typename Derived>
/**
 * @brief Cross Product Matrix.
 * @param [in] in: In.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> CrossProductMatrix(const Eigen::DenseBase<Derived>& in) {
  if (in.innerSize() != 3 || in.outerSize() != 1) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("Input argument should be a 3-by-1 vector.");
  }

  Eigen::Matrix<SCALAR_T, 3, 3> out;
  /**
   * @brief SCALAR T.
   * @return Result value.
   */
  Out << SCALAR_T(0.0), -in(2), +in(1), +in(2), SCALAR_T(0.0), -in(0), -in(1), +in(0), SCALAR_T(0.0);
  return out;
}

/**
 * Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
 * @param [in] eulerAngles: euler angles in xyz convention
 * @return M: matrix that does the transformation
 */
template <typename SCALAR_T>
/**
 * @brief Angular Velocities To Euler Angle Derivatives Matrix.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> AngularVelocitiesToEulerAngleDerivativesMatrix(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  Eigen::Matrix<SCALAR_T, 3, 3> M;
  SCALAR_T sinPsi = sin(eulerAngles(2));
  SCALAR_T cosPsi = cos(eulerAngles(2));
  SCALAR_T sinTheta = sin(eulerAngles(1));
  SCALAR_T cosTheta = cos(eulerAngles(1));

  M << cosPsi / cosTheta, -sinPsi / cosTheta, SCALAR_T(0), sinPsi, cosPsi, SCALAR_T(0), -cosPsi * sinTheta / cosTheta,
      sinTheta * sinPsi / cosTheta, SCALAR_T(1);

  return M;
}

template <typename SCALAR_T>
/**
 * @brief Angular Velocities To Euler Angle Derivatives.
 * @param [in] angularVelocitiesInBase: Angular Velocities In Base.
 * @param [in] eulerAngles: Euler Angles.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> AngularVelocitiesToEulerAngleDerivatives(const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocitiesInBase,
                                                                       const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T sinPsi = sin(eulerAngles(2));
  const SCALAR_T cosPsi = cos(eulerAngles(2));
  const SCALAR_T sinTheta = sin(eulerAngles(1));
  const SCALAR_T cosTheta = cos(eulerAngles(1));

  const SCALAR_T dRoll = (cosPsi * angularVelocitiesInBase(0) - sinPsi * angularVelocitiesInBase(1)) / cosTheta;
  const SCALAR_T dPitch = sinPsi * angularVelocitiesInBase(0) + cosPsi * angularVelocitiesInBase(1);
  const SCALAR_T dYaw = angularVelocitiesInBase(2) - sinTheta * dRoll;

  return {dRoll, dPitch, dYaw};
}

/**
 * Computes the derivative w.r.t eulerXYZ of rotating a vector from base to origin : d/d(euler) (o_R_b(euler) * v_base)
 *
 * @param [in] eulerAnglesXYZ: euler angles in xyz convention.
 * @param [in] vectorInBase: the vector in base that is rotated.
 * @return d/d(euler) (o_R_b(euler) * v_base)
 */
template <typename SCALAR_T>
/**
 * @brief Rotation Base To Origin Jacobian.
 * @param [in] eulerAnglesXYZ: Euler Angles XYZ.
 * @param [in] vectorInBase: Vector In Base.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 3> RotationBaseToOriginJacobian(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ,
                                                           const Eigen::Matrix<SCALAR_T, 3, 1>& vectorInBase) {
  // dependent variables
  Eigen::Matrix<SCALAR_T, 3, 3> jac;

  // auxiliary variables
  std::array<SCALAR_T, 12> v{};

  /**
   * @brief Constructs a Jac.
   */
  Jac(0) = SCALAR_T(0.0);
  v[0] = -sin(eulerAnglesXYZ[1]);
  v[1] = cos(eulerAnglesXYZ[2]);
  v[2] = sin(eulerAnglesXYZ[2]);
  v[3] = v[1] * vectorInBase[0] - v[2] * vectorInBase[1];
  v[4] = cos(eulerAnglesXYZ[1]);
  Jac(3) = v[0] * v[3] + v[4] * vectorInBase[2];
  v[5] = v[4];
  v[6] = -v[2];
  v[7] = v[1];
  v[8] = v[6] * vectorInBase[0] - v[7] * vectorInBase[1];
  Jac(6) = v[5] * v[8];
  v[9] = -sin(eulerAnglesXYZ[0]);
  v[2] = v[2] * vectorInBase[0] + v[1] * vectorInBase[1];
  v[1] = cos(eulerAnglesXYZ[0]);
  v[10] = v[0];
  v[5] = v[10] * v[3] + v[5] * vectorInBase[2];
  Jac(1) = v[9] * v[2] - v[1] * v[5];
  v[11] = -v[9];
  v[4] = (-v[4]) * v[3] + v[0] * vectorInBase[2];
  Jac(4) = -v[11] * v[4];
  v[3] = v[1];
  v[7] = v[7] * vectorInBase[0] + v[6] * vectorInBase[1];
  v[10] = v[10] * v[8];
  Jac(7) = v[3] * v[7] - v[11] * v[10];
  Jac(2) = v[1] * v[2] + v[9] * v[5];
  Jac(5) = v[3] * v[4];
  Jac(8) = v[11] * v[7] + v[3] * v[10];
  return jac;
}

template <typename SCALAR_T>
/**
 * @brief Rotation Error In World Euler XYZ.
 * @param [in] eulerXYZcurrent: Euler XYZcurrent.
 * @param [in] eulerXYZreference: Euler XYZreference.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotationErrorInWorldEulerXYZ(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZcurrent,
                                                           const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZreference) {
  /**
   * @brief Rotation Error In World.
   * @return Result value.
   */
  return bolza::RotationErrorInWorld(RotationMatrixBaseToOrigin(eulerXYZcurrent), RotationMatrixBaseToOrigin(eulerXYZreference));
}

template <typename SCALAR_T>
/**
 * @brief Rotation Error In Local Euler XYZ.
 * @param [in] eulerXYZcurrent: Euler XYZcurrent.
 * @param [in] eulerXYZreference: Euler XYZreference.
 * @return Result value.
 */
Eigen::Matrix<SCALAR_T, 3, 1> RotationErrorInLocalEulerXYZ(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZcurrent,
                                                           const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZreference) {
  /**
   * @brief Rotation Error In Local.
   * @return Result value.
   */
  return bolza::RotationErrorInLocal(RotationMatrixBaseToOrigin(eulerXYZcurrent), RotationMatrixBaseToOrigin(eulerXYZreference));
}

}  // namespace switched_model
