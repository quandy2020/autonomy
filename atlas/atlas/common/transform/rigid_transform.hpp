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
 * @file rigid_transform.hpp
 * @brief @ref Rigid2 / @ref Rigid3 pose types (Cartographer
 *        `transform/rigid_transform` port).
 * No Lua or protobuf dependencies. Argus uses @ref Rigid3d as the system pose
 * type.
 */

#ifndef ATLAS_COMMON_RIGID_TRANSFORM_HPP_
#define ATLAS_COMMON_RIGID_TRANSFORM_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace atlas {
namespace transform {

/**
 * @brief Wrap an angle difference into the interval \([-\pi, \pi]\).
 * @tparam T Scalar type.
 * @param[in] difference Raw angle difference in radians.
 * @return Equivalent angle in \([-\pi, \pi]\).
 */
template <typename T>
T NormalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) {
    difference -= T(2.) * kPi;
  }
  while (difference < -kPi) {
    difference += T(2.) * kPi;
  }
  return difference;
}

/**
 * @brief 2D rigid transform: planar translation plus rotation about +Z.
 * @tparam FloatType Scalar type (`float` or `double`).
 */
template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  /**
   * @brief Identity pose (zero translation, zero rotation).
   */
  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}

  /**
   * @brief Construct from translation and a 2D rotation object.
   * @param[in] translation Planar translation.
   * @param[in] rotation    Planar rotation.
   */
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * @brief Construct from translation and a scalar angle.
   * @param[in] translation Planar translation.
   * @param[in] rotation    Rotation angle in radians.
   */
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * @brief Pure rotation about the origin.
   * @param[in] rotation Angle in radians.
   * @return Transform with zero translation.
   */
  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  /**
   * @brief Pure rotation about the origin.
   * @param[in] rotation 2D rotation object.
   * @return Transform with zero translation.
   */
  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  /**
   * @brief Pure translation (identity rotation).
   * @param[in] vector Translation vector.
   * @return Transform with identity rotation.
   */
  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  /**
   * @brief Identity transform factory.
   * @return Pose with zero translation and identity rotation.
   */
  static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }

  /**
   * @brief Cast translation and rotation to another scalar type.
   * @tparam OtherType Destination scalar type.
   * @return Pose with cast components.
   */
  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  /**
   * @brief Translation component.
   * @return Const reference to the 2D translation vector.
   */
  const Vector& translation() const { return translation_; }

  /**
   * @brief Planar rotation component.
   * @return Copy of the `Rotation2D`.
   */
  Rotation2D rotation() const { return rotation_; }

  /**
   * @brief Rotation angle wrapped to \([-\pi, \pi]\).
   * @return Normalized angle in radians.
   */
  double normalized_angle() const {
    return NormalizeAngleDifference(rotation().angle());
  }

  /**
   * @brief Inverse rigid transform.
   * @return \(T^{-1}\) such that `*this * inverse()` is identity.
   */
  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  /**
   * @brief Human-readable debug string.
   * @return Text of the form `{ t: [x, y], r: [angle] }`.
   */
  std::string DebugString() const {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "{ t: [%f, %f], r: [%f] }",
                  translation().x(), translation().y(), rotation().angle());
    return std::string(buf);
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

/**
 * @brief Compose two planar rigid transforms: apply @p rhs first.
 * @tparam FloatType Scalar type.
 * @param[in] lhs Left-hand transform.
 * @param[in] rhs Right-hand transform.
 * @return Composition \(\mathrm{lhs} \circ \mathrm{rhs}\).
 */
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

/**
 * @brief Transform a 2D point by a planar rigid pose.
 * @tparam FloatType Scalar type.
 * @param[in] rigid Input pose.
 * @param[in] point Point in the source frame.
 * @return Transformed point \(R p + t\).
 */
template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

/**
 * @brief Stream @ref Rigid2 via @ref Rigid2::DebugString.
 * @tparam T Scalar type.
 * @param[in,out] os    Output stream.
 * @param[in]     rigid Pose to print.
 * @return Reference to @p os.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

/** @brief Double-precision planar rigid transform. */
using Rigid2d = Rigid2<double>;

/** @brief Single-precision planar rigid transform. */
using Rigid2f = Rigid2<float>;

/**
 * @brief 3D rigid transform: translation plus unit-quaternion rotation.
 * @tparam FloatType Scalar type (`float` or `double`).
 */
template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  /**
   * @brief Identity pose.
   */
  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}

  /**
   * @brief Construct from translation and quaternion.
   * @param[in] translation 3D translation (metres).
   * @param[in] rotation    Unit quaternion (not re-normalized here).
   */
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * @brief Construct from translation and angle-axis.
   * @param[in] translation 3D translation (metres).
   * @param[in] rotation    Angle-axis rotation.
   */
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * @brief Pure rotation (zero translation) from angle-axis.
   * @param[in] angle_axis Axis-angle rotation.
   * @return Transform with zero translation.
   */
  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  /**
   * @brief Pure rotation (zero translation) from a quaternion.
   * @param[in] rotation Unit quaternion.
   * @return Transform with zero translation.
   */
  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  /**
   * @brief Pure translation (identity rotation).
   * @param[in] vector Translation vector (metres).
   * @return Transform with identity quaternion.
   */
  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  /**
   * @brief Build from quaternion \((w,x,y,z)\) and translation arrays.
   * @param[in] rotation    Four quaternion components `[w, x, y, z]`.
   * @param[in] translation Three translation components `[x, y, z]`.
   * @return Constructed @ref Rigid3 pose.
   */
  static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                           const std::array<FloatType, 3>& translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                               rotation[2], rotation[3]));
  }

  /**
   * @brief Identity transform factory.
   * @return Pose with zero translation and identity quaternion.
   */
  static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

  /**
   * @brief Cast components to another scalar type.
   * @tparam OtherType Destination scalar type.
   * @return Pose with cast translation and rotation.
   */
  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  /**
   * @brief Translation component.
   * @return Const reference to the 3D translation vector.
   */
  const Vector& translation() const { return translation_; }

  /**
   * @brief Rotation as a unit quaternion.
   * @return Const reference to the quaternion.
   */
  const Quaternion& rotation() const { return rotation_; }

  /**
   * @brief Inverse rigid transform.
   * @return \(T^{-1}\) with conjugated quaternion and transformed
   *         translation.
   */
  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  /**
   * @brief Human-readable debug string.
   * @return Text of the form `{ t: [...], q: [w, x, y, z] }`.
   */
  std::string DebugString() const {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "{ t: [%f, %f, %f], q: [%f, %f, %f, %f] }",
                  translation().x(), translation().y(), translation().z(),
                  rotation().w(), rotation().x(), rotation().y(),
                  rotation().z());
    return std::string(buf);
  }

  /**
   * @brief Check that translation is finite and the quaternion is near unit.
   * @return `false` if any translation component is NaN or
   *         \(|1 - \\|q\\|| \ge 10^{-3}\); otherwise `true`.
   */
  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

/**
 * @brief Compose two 3D rigid transforms: apply @p rhs first.
 * @tparam FloatType Scalar type.
 * @param[in] lhs Left-hand transform.
 * @param[in] rhs Right-hand transform.
 * @return Composition \(\mathrm{lhs} \circ \mathrm{rhs}\) with renormalized
 *         quaternion.
 */
template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

/**
 * @brief Transform a 3D point by a rigid pose.
 * @tparam FloatType Scalar type.
 * @param[in] rigid Input pose.
 * @param[in] point Point in the source frame.
 * @return Transformed point \(R p + t\).
 */
template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

/**
 * @brief Stream @ref Rigid3 via @ref Rigid3::DebugString.
 * @tparam T Scalar type.
 * @param[in,out] os    Output stream.
 * @param[in]     rigid Pose to print.
 * @return Reference to @p os.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

/** @brief Double-precision 3D rigid transform (system pose type). */
using Rigid3d = Rigid3<double>;

/** @brief Single-precision 3D rigid transform. */
using Rigid3f = Rigid3<float>;

/**
 * @brief Build a unit quaternion from URDF-style fixed-axis ZYX Euler angles.
 * Composition order is yaw × pitch × roll (about fixed X/Y/Z respectively).
 * @param[in] roll  Rotation about fixed X (radians).
 * @param[in] pitch Rotation about fixed Y (radians).
 * @param[in] yaw   Rotation about fixed Z (radians).
 * @return Unit quaternion representing the composed rotation.
 */
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

}  // namespace transform
}  // namespace atlas

#endif  // ATLAS_COMMON_RIGID_TRANSFORM_HPP_
