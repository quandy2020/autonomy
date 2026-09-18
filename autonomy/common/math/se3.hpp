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
 * @brief Special Euclidean group SE(3).
 *
 * Templated rigid transform (rotation + translation) styled after
 * `autonomy::common::transform::Rigid3`. Storage is a 3-vector translation
 * and a unit quaternion. Constructor argument order is
 * `(translation, rotation)`, matching `Rigid3`.
 *
 * Lie-algebra maps `FromTwist` / `ToTwist` implement the standard SE(3)
 * exponential / logarithm with the left Jacobian of SO(3). Rotation-vector
 * conversions go through `SO3`.
 *
 */

#pragma once

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "autonomy/common/math/so3.hpp"
#include "autonomy/common/string_util.hpp"

namespace autonomy {
namespace common {
namespace math {

/**
 * @class SE3
 * @brief Element of the Special Euclidean group SE(3).
 *
 * Represents a rigid body transform
 * @f[
 *   T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix},
 *   \quad R \in \mathrm{SO}(3),\; t \in \mathbb{R}^3.
 * @f]
 *
 * Composition and point action are free `operator*` overloads, matching
 * `autonomy::common::transform::Rigid3`.
 *
 * @tparam FloatType Scalar type, typically `double` or `float`.
 */
template <typename FloatType>
class SE3 {
 public:
  /**
   * @brief 3-vector over `FloatType`.
   *
   * Used for translation vectors and 3D points.
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
   * @brief 3x3 matrix type for rotation and Jacobian blocks.
   */
  using Matrix3x3 = Eigen::Matrix<FloatType, 3, 3>;

  /**
   * @brief 4x4 homogeneous transform matrix type.
   */
  using Matrix4x4 = Eigen::Matrix<FloatType, 4, 4>;

  /**
   * @brief 6-vector twist coordinates.
   *
   * Layout is
   * @f$ \xi = [\omega^{\mathsf{T}},\, v^{\mathsf{T}}]^{\mathsf{T}} @f$.
   */
  using TwistVector = Eigen::Matrix<FloatType, 6, 1>;

  /**
   * @brief Constructs the identity transform.
   *
   * Zero translation and identity rotation.
   */
  SE3()
      : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}

  /**
   * @brief Constructs from translation and quaternion.
   * @param translation Translation vector @f$ t @f$.
   * @param rotation Rotation quaternion (prefer unit length).
   *
   * Argument order matches `Rigid3`: translation first, then rotation.
   */
  SE3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * @brief Constructs from translation and angle-axis.
   * @param translation Translation vector @f$ t @f$.
   * @param rotation Angle-axis rotation.
   */
  SE3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * @brief Constructs from translation and a 3x3 rotation matrix.
   * @param translation Translation vector @f$ t @f$.
   * @param rotation Rotation matrix; converted to a normalized quaternion.
   */
  SE3(const Vector& translation, const Matrix3x3& rotation)
      : translation_(translation), rotation_(rotation) {
    rotation_.normalize();
  }

  /**
   * @brief Constructs from translation and an SO(3) rotation.
   * @param translation Translation vector @f$ t @f$.
   * @param rotation SO(3) element whose quaternion is copied.
   */
  SE3(const Vector& translation, const SO3<FloatType>& rotation)
      : translation_(translation), rotation_(rotation.Rotation()) {}

  /**
   * @brief Pure rotation about the origin from an angle-axis.
   * @param angle_axis Rotation angle in radians and unit axis.
   * @return Transform with zero translation.
   */
  static SE3 FromAngleAxis(const AngleAxis& angle_axis) {
    return SE3(Vector::Zero(), Quaternion(angle_axis));
  }

  /**
   * @brief Pure rotation about the origin from a quaternion.
   * @param rotation Rotation quaternion.
   * @return Transform with zero translation.
   */
  static SE3 FromQuaternion(const Quaternion& rotation) {
    return SE3(Vector::Zero(), rotation);
  }

  /**
   * @brief Pure translation with identity rotation.
   * @param translation Translation vector @f$ t @f$.
   * @return Transform with identity rotation.
   */
  static SE3 FromTranslation(const Vector& translation) {
    return SE3(translation, Quaternion::Identity());
  }

  /**
   * @brief Returns the identity element of SE(3).
   * @return Identity rigid transform.
   */
  static SE3 Identity() { return SE3(); }

  /**
   * @brief Exponential map from a twist to SE(3).
   * @param twist Twist coordinates
   *        @f$ \xi = [\omega^{\mathsf{T}},\, v^{\mathsf{T}}]^{\mathsf{T}} @f$,
   *        where @f$ \omega @f$ is the rotation vector and @f$ v @f$ is the
   *        translational component of the twist.
   * @return Rigid transform @f$ \exp(\hat{\xi}) @f$.
   *
   * Uses @f$ R = \mathrm{Exp}(\omega) @f$ via `SO3::FromRotationVector`
   * and the left Jacobian @f$ J_l(\omega) @f$ so that
   * @f$ t = J_l(\omega)\, v @f$.
   * For @f$ \|\omega\| \approx 0 @f$, @f$ J_l = I @f$.
   */
  static SE3 FromTwist(const TwistVector& twist) {
    const Vector rotation_vector = twist.template head<3>();
    const Vector translational_velocity = twist.template tail<3>();
    const SO3<FloatType> rotation =
        SO3<FloatType>::FromRotationVector(rotation_vector);
    const FloatType rotation_angle = rotation_vector.norm();
    Matrix3x3 left_jacobian = Matrix3x3::Identity();
    if (rotation_angle > FloatType(1e-10)) {
      const Matrix3x3 skew_symmetric_rotation =
          SkewSymmetricMatrix(rotation_vector);
      left_jacobian =
          Matrix3x3::Identity() +
          (FloatType(1) - std::cos(rotation_angle)) /
              (rotation_angle * rotation_angle) * skew_symmetric_rotation +
          (rotation_angle - std::sin(rotation_angle)) /
              (rotation_angle * rotation_angle * rotation_angle) *
              skew_symmetric_rotation * skew_symmetric_rotation;
    }
    return SE3(left_jacobian * translational_velocity, rotation);
  }

  /**
   * @brief Logarithmic map from SE(3) to a twist.
   * @return Twist coordinates
   *         @f$ \xi = [\omega^{\mathsf{T}},\, v^{\mathsf{T}}]^{\mathsf{T}} @f$
   *         with @f$ \omega = \mathrm{Log}(R) @f$ and
   *         @f$ v = J_l(\omega)^{-1}\, t @f$.
   *
   * For @f$ \|\omega\| \approx 0 @f$, @f$ J_l^{-1} = I @f$.
   */
  TwistVector ToTwist() const {
    TwistVector twist;
    const Vector rotation_vector = ToSO3().ToRotationVector();
    twist.template head<3>() = rotation_vector;
    const FloatType rotation_angle = rotation_vector.norm();
    Matrix3x3 left_jacobian_inverse = Matrix3x3::Identity();
    if (rotation_angle > FloatType(1e-10)) {
      const Matrix3x3 skew_symmetric_rotation =
          SkewSymmetricMatrix(rotation_vector);
      left_jacobian_inverse =
          Matrix3x3::Identity() - FloatType(0.5) * skew_symmetric_rotation +
          (FloatType(1) / (rotation_angle * rotation_angle) -
           (FloatType(1) + std::cos(rotation_angle)) /
               (FloatType(2) * rotation_angle * std::sin(rotation_angle))) *
              skew_symmetric_rotation * skew_symmetric_rotation;
    }
    twist.template tail<3>() = left_jacobian_inverse * translation_;
    return twist;
  }

  /**
   * @brief Casts the transform to another scalar type.
   * @tparam OtherType Destination scalar type.
   * @return SE(3) element with `OtherType` coefficients.
   */
  template <typename OtherType>
  SE3<OtherType> Cast() const {
    return SE3<OtherType>(translation_.template cast<OtherType>(),
                          rotation_.template cast<OtherType>());
  }

  /**
   * @brief Accesses the translation vector @f$ t @f$.
   * @return Const reference to the translation.
   */
  const Vector& Translation() const { return translation_; }

  /**
   * @brief Accesses the rotation quaternion.
   * @return Const reference to the stored unit quaternion.
   */
  const Quaternion& Rotation() const { return rotation_; }

  /**
   * @brief Converts the rotation part to a 3x3 matrix.
   * @return Orthogonal matrix @f$ R @f$.
   */
  Matrix3x3 RotationMatrix() const { return rotation_.toRotationMatrix(); }

  /**
   * @brief Views the rotation part as an SO(3) element.
   * @return SO(3) wrapping the stored quaternion.
   */
  SO3<FloatType> ToSO3() const { return SO3<FloatType>(rotation_); }

  /**
   * @brief Returns the group inverse.
   *
   * Computes
   * @f$ T^{-1} = (R^{\mathsf{T}},\, -R^{\mathsf{T}} t) @f$.
   *
   * @return Inverse rigid transform.
   */
  SE3 Inverse() const {
    const Quaternion inverse_rotation = rotation_.conjugate();
    const Vector inverse_translation = -(inverse_rotation * translation_);
    return SE3(inverse_translation, inverse_rotation);
  }

  /**
   * @brief Converts to a 4x4 homogeneous matrix.
   * @return Matrix @f$ \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} @f$.
   */
  Matrix4x4 Matrix() const {
    Matrix4x4 homogeneous_transform = Matrix4x4::Identity();
    homogeneous_transform.template block<3, 3>(0, 0) = RotationMatrix();
    homogeneous_transform.template block<3, 1>(0, 3) = translation_;
    return homogeneous_transform;
  }

  /**
   * @brief Checks finiteness of translation and unit norm of the quaternion.
   * @return True if no component of @f$ t @f$ is NaN and
   *         @f$ |1 - \|q\|| < 10^{-3} @f$.
   */
  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }

  /**
   * @brief Formats translation and quaternion for logging and debugging.
   * @return String of the form `{ t: [x, y, z], q: [w, x, y, z] }`.
   */
  std::string DebugString() const {
    return common::StringPrintf(
        "{ t: [%f, %f, %f], q: [%f, %f, %f, %f] }", Translation().x(),
        Translation().y(), Translation().z(), Rotation().w(), Rotation().x(),
        Rotation().y(), Rotation().z());
  }

 private:
  /**
   * @brief Translation vector @f$ t \in \mathbb{R}^3 @f$.
   */
  Vector translation_;

  /**
   * @brief Unit quaternion representing @f$ R \in \mathrm{SO}(3) @f$.
   */
  Quaternion rotation_;
};

/**
 * @brief Composes two rigid transforms.
 *
 * Computes @f$ T_{\mathrm{left}} T_{\mathrm{right}} @f$:
 * @f[
 *   t = R_{\mathrm{left}} t_{\mathrm{right}} + t_{\mathrm{left}},\quad
 *   R = R_{\mathrm{left}} R_{\mathrm{right}}.
 * @f]
 *
 * @tparam FloatType Scalar type.
 * @param left Left operand.
 * @param right Right operand.
 * @return Product with a renormalized quaternion.
 */
template <typename FloatType>
SE3<FloatType> operator*(const SE3<FloatType>& left,
                         const SE3<FloatType>& right) {
  return SE3<FloatType>(
      left.Rotation() * right.Translation() + left.Translation(),
      (left.Rotation() * right.Rotation()).normalized());
}

/**
 * @brief Transforms a 3D point.
 *
 * Computes @f$ T\,p = R\,p + t @f$.
 *
 * @tparam FloatType Scalar type.
 * @param transform SE(3) element.
 * @param point Point in @f$ \mathbb{R}^3 @f$.
 * @return Transformed point.
 */
template <typename FloatType>
typename SE3<FloatType>::Vector operator*(
    const SE3<FloatType>& transform,
    const typename SE3<FloatType>::Vector& point) {
  return transform.Rotation() * point + transform.Translation();
}

/**
 * @brief Streams DebugString() for gmock and logging.
 * @tparam FloatType Scalar type.
 * @param stream Output stream.
 * @param transform Value to print.
 * @return Reference to `stream`.
 */
template <typename FloatType>
std::ostream& operator<<(std::ostream& stream,
                         const SE3<FloatType>& transform) {
  stream << transform.DebugString();
  return stream;
}

/**
 * @brief Double-precision SE(3).
 */
using SE3d = SE3<double>;

/**
 * @brief Single-precision SE(3).
 */
using SE3f = SE3<float>;

}  // namespace math
}  // namespace common
}  // namespace autonomy

