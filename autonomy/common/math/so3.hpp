/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief SO(3) helpers and the templated SO(3) group element.
 */

#pragma once

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "autonomy/common/string_util.hpp"

namespace autonomy {
namespace common {
namespace math {

/**
 * @brief Builds the 3x3 skew-symmetric matrix of a 3-vector.
 * @tparam Derived Eigen 3-vector expression type.
 * @param vector Input 3-vector.
 * @return Skew-symmetric matrix [v]_x.
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetricMatrix(
    const Eigen::MatrixBase<Derived>& vector) {
  using Scalar = typename Derived::Scalar;
  Eigen::Matrix<Scalar, 3, 3> skew;
  skew << Scalar(0), -vector(2), vector(1), vector(2), Scalar(0), -vector(0),
      -vector(1), vector(0), Scalar(0);
  return skew;
}

/**
 * @brief Builds the skew-symmetric matrix from three scalar components.
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> SkewSymmetricMatrix(const Scalar& v0,
                                                const Scalar& v1,
                                                const Scalar& v2) {
  return SkewSymmetricMatrix(Eigen::Matrix<Scalar, 3, 1>(v0, v1, v2));
}

/**
 * @brief Angle-axis vector → unit quaternion (with near-zero linearization).
 */
template <typename Scalar>
Eigen::Quaternion<Scalar> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<Scalar, 3, 1>& angle_axis) {
  Scalar scale = Scalar(0.5);
  Scalar w = Scalar(1);
  constexpr Scalar kCutoffAngle = Scalar(1e-8);
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const Scalar norm = angle_axis.norm();
    scale = std::sin(norm / Scalar(2)) / norm;
    w = std::cos(norm / Scalar(2));
  }
  const Eigen::Matrix<Scalar, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<Scalar>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                                   quaternion_xyz.z());
}

/**
 * @brief Unit quaternion → angle-axis vector (positive-w hemisphere).
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<Scalar>& quaternion) {
  Eigen::Quaternion<Scalar> normalized_quaternion = quaternion.normalized();
  // Prefer the hemisphere with non-negative w (smaller angle).
  if (normalized_quaternion.w() < Scalar(0)) {
    normalized_quaternion.coeffs() *= Scalar(-1);
  }
  const Scalar angle =
      Scalar(2) * std::atan2(normalized_quaternion.vec().norm(),
                             normalized_quaternion.w());
  constexpr Scalar kCutoffAngle = Scalar(1e-7);
  const Scalar scale =
      angle < kCutoffAngle ? Scalar(2) : angle / std::sin(angle / Scalar(2));
  return scale * normalized_quaternion.vec();
}

/**
 * @brief SO(3) exponential map: rotation vector → rotation matrix.
 * @param rotation_vector Angle-axis vector (radians · axis).
 * @return Rotation matrix R = Exp([ω]_x).
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> RotationVectorToRotationMatrix(
    const Eigen::Matrix<Scalar, 3, 1>& rotation_vector) {
  return AngleAxisVectorToRotationQuaternion(rotation_vector)
      .toRotationMatrix();
}

/**
 * @brief SO(3) logarithm map: rotation matrix → rotation vector.
 * @param rotation Rotation matrix in SO(3).
 * @return Angle-axis vector (radians · axis).
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> RotationMatrixToRotationVector(
    const Eigen::Matrix<Scalar, 3, 3>& rotation) {
  return RotationQuaternionToAngleAxisVector(
      Eigen::Quaternion<Scalar>(rotation));
}

/**
 * @brief Extracts ZYX (yaw-pitch-roll) Euler angles from a rotation matrix.
 *
 * Returns (roll, pitch, yaw) with the same convention historically used by
 * FAST-LIVO2 / LIVO (`RotMtoEuler`). Distinct from `EulerAnglesZXY`.
 *
 * @param rotation Rotation matrix.
 * @return Vector3 (roll, pitch, yaw) in radians.
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> RotationMatrixToRollPitchYaw(
    const Eigen::Matrix<Scalar, 3, 3>& rotation) {
  const Scalar sine_yaw =
      std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
  const bool singular = sine_yaw < Scalar(1e-6);
  Scalar roll;
  Scalar pitch;
  Scalar yaw;
  if (!singular) {
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    pitch = std::atan2(-rotation(2, 0), sine_yaw);
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    roll = std::atan2(-rotation(1, 2), rotation(1, 1));
    pitch = std::atan2(-rotation(2, 0), sine_yaw);
    yaw = Scalar(0);
  }
  return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
}

/**
 * @class SO3
 * @brief Element of the Special Orthogonal group SO(3).
 *
 * Internally stores a unit quaternion. Constructors that take a rotation
 * representation normalize the quaternion. Composition and point action are
 * free `operator*` overloads, matching
 * `autonomy::common::transform::Rigid3`.
 *
 * @tparam FloatType Scalar type, typically `double` or `float`.
 */
template <typename FloatType>
class SO3 {
 public:
  /**
   * @brief 3-vector over `FloatType`.
   *
   * Used for rotation vectors and 3D points.
   */
  using Vector = Eigen::Matrix<FloatType, 3, 1>;

  /**
   * @brief Unit quaternion type used to store the rotation.
   */
  using Quaternion = Eigen::Quaternion<FloatType>;

  /**
   * @brief Eigen angle-axis representation.
   */
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  /**
   * @brief 3x3 rotation matrix type.
   */
  using Matrix = Eigen::Matrix<FloatType, 3, 3>;

  /**
   * @brief Constructs the identity rotation.
   *
   * Equivalent to the unit quaternion `(w, x, y, z) = (1, 0, 0, 0)`.
   */
  SO3() : rotation_(Quaternion::Identity()) {}

  /**
   * @brief Constructs from a quaternion and normalizes it.
   * @param rotation Input quaternion (need not be unit length).
   */
  explicit SO3(const Quaternion& rotation) : rotation_(rotation) {
    rotation_.normalize();
  }

  /**
   * @brief Constructs from a 3x3 rotation matrix and normalizes the result.
   * @param rotation Orthogonal rotation matrix (approximately).
   *
   * Numerical noise in `rotation` is absorbed by quaternion normalization
   * after conversion.
   */
  explicit SO3(const Matrix& rotation) : rotation_(rotation) {
    rotation_.normalize();
  }

  /**
   * @brief Constructs from an Eigen angle-axis and normalizes the result.
   * @param angle_axis Rotation angle in radians and unit axis.
   */
  explicit SO3(const AngleAxis& angle_axis) : rotation_(angle_axis) {
    rotation_.normalize();
  }

  /**
   * @brief Returns the identity element of SO(3).
   * @return Identity rotation.
   */
  static SO3 Identity() { return SO3(); }

  /**
   * @brief Factory from a quaternion.
   * @param rotation Input quaternion.
   * @return Normalized SO(3) element.
   */
  static SO3 FromQuaternion(const Quaternion& rotation) {
    return SO3(rotation);
  }

  /**
   * @brief Factory from an angle-axis.
   * @param angle_axis Rotation angle in radians and unit axis.
   * @return Normalized SO(3) element.
   */
  static SO3 FromAngleAxis(const AngleAxis& angle_axis) {
    return SO3(angle_axis);
  }

  /**
   * @brief Exponential map from a rotation vector to SO(3).
   * @param rotation_vector Angle-axis vector whose norm is the rotation
   *        angle in radians and whose direction is the rotation axis.
   * @return Corresponding SO(3) rotation.
   *
   * Delegates to
   * `autonomy::AngleAxisVectorToRotationQuaternion`.
   * Near-zero angles are linearized by that helper.
   */
  static SO3 FromRotationVector(const Vector& rotation_vector) {
    return SO3(
        AngleAxisVectorToRotationQuaternion(rotation_vector));
  }

  /**
   * @brief Logarithmic map from SO(3) to a rotation vector.
   * @return Angle-axis vector (radians times axis).
   *
   * Delegates to
   * `autonomy::RotationQuaternionToAngleAxisVector`,
   * which selects the quaternion hemisphere with non-negative `w`.
   */
  Vector ToRotationVector() const {
    return RotationQuaternionToAngleAxisVector(rotation_);
  }

  /**
   * @brief Casts the rotation to another scalar type.
   * @tparam OtherType Destination scalar type.
   * @return SO(3) element with `OtherType` coefficients.
   */
  template <typename OtherType>
  SO3<OtherType> Cast() const {
    return SO3<OtherType>(rotation_.template cast<OtherType>());
  }

  /**
   * @brief Accesses the stored unit quaternion.
   * @return Const reference to the internal quaternion.
   */
  const Quaternion& Rotation() const { return rotation_; }

  /**
   * @brief Converts to a 3x3 rotation matrix.
   * @return Orthogonal matrix @f$ R \in \mathrm{SO}(3) @f$.
   */
  Matrix Matrix() const { return rotation_.toRotationMatrix(); }

  /**
   * @brief Alias of Matrix() for call sites that expect a rotation matrix.
   * @return Orthogonal matrix @f$ R \in \mathrm{SO}(3) @f$.
   */
  Matrix RotationMatrix() const { return Matrix(); }

  /**
   * @brief Returns the group inverse @f$ R^{-1} = R^{\mathsf{T}} @f$.
   * @return Inverse rotation (quaternion conjugate).
   */
  SO3 Inverse() const { return SO3(rotation_.conjugate()); }

  /**
   * @brief Checks that the stored quaternion is approximately unit length.
   * @return True if @f$ |1 - \|q\|| < 10^{-3} @f$.
   */
  bool IsValid() const {
    return std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }

  /**
   * @brief Formats the quaternion for logging and debugging.
   * @return String of the form `{ q: [w, x, y, z] }`.
   */
  std::string DebugString() const {
    return common::StringPrintf("{ q: [%f, %f, %f, %f] }", Rotation().w(),
                                Rotation().x(), Rotation().y(),
                                Rotation().z());
  }

 private:
  /**
   * @brief Unit quaternion representing the rotation.
   */
  Quaternion rotation_;
};

/**
 * @brief Composes two SO(3) rotations.
 *
 * Computes @f$ R_{\mathrm{left}} R_{\mathrm{right}} @f$.
 *
 * @tparam FloatType Scalar type.
 * @param left Left operand.
 * @param right Right operand.
 * @return Product with a renormalized quaternion.
 */
template <typename FloatType>
SO3<FloatType> operator*(const SO3<FloatType>& left,
                         const SO3<FloatType>& right) {
  return SO3<FloatType>(
      (left.Rotation() * right.Rotation()).normalized());
}

/**
 * @brief Applies a rotation to a 3D point.
 *
 * Computes @f$ R\,p @f$.
 *
 * @tparam FloatType Scalar type.
 * @param rotation SO(3) element.
 * @param point Point in @f$ \mathbb{R}^3 @f$.
 * @return Rotated point.
 */
template <typename FloatType>
typename SO3<FloatType>::Vector operator*(
    const SO3<FloatType>& rotation,
    const typename SO3<FloatType>::Vector& point) {
  return rotation.Rotation() * point;
}

/**
 * @brief Streams DebugString() for gmock and logging.
 * @tparam FloatType Scalar type.
 * @param stream Output stream.
 * @param rotation Value to print.
 * @return Reference to `stream`.
 */
template <typename FloatType>
std::ostream& operator<<(std::ostream& stream, const SO3<FloatType>& rotation) {
  stream << rotation.DebugString();
  return stream;
}

/**
 * @brief Double-precision SO(3).
 */
using SO3d = SO3<double>;

/**
 * @brief Single-precision SO(3).
 */
using SO3f = SO3<float>;


}  // namespace math
}  // namespace common
}  // namespace autonomy
