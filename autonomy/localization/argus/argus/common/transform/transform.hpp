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
 * @file transform.hpp
 * @brief Rigid transform helpers and Eigen Isometry bridges
 * (Cartographer-style).
 */

#ifndef ARGUS_COMMON_TRANSFORM_HPP_
#define ARGUS_COMMON_TRANSFORM_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace transform {

/**
 * @brief Rotation angle magnitude of a rigid pose.
 * @tparam FloatType Scalar type of @ref Rigid3.
 * @param[in] transform Input pose.
 * @return Angle in radians in \([0, \pi]\) derived from the quaternion.
 */
template <typename FloatType>
FloatType GetAngle(const Rigid3<FloatType>& transform) {
  return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                   std::abs(transform.rotation().w()));
}

/**
 * @brief Yaw about the world +Z axis from a unit quaternion.
 * @tparam T Scalar type.
 * @param[in] rotation Unit quaternion (normalized preferred).
 * @return Yaw angle in radians.
 */
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) {
  const Eigen::Matrix<T, 3, 1> direction =
      rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return std::atan2(direction.y(), direction.x());
}

/**
 * @brief Yaw about the world +Z axis from a @ref Rigid3 pose.
 * @tparam T Scalar type.
 * @param[in] transform Input 3D pose.
 * @return Yaw angle in radians (same as @ref GetYaw for its quaternion).
 */
template <typename T>
T GetYaw(const Rigid3<T>& transform) {
  return GetYaw(transform.rotation());
}

/**
 * @brief Convert a quaternion to an angle-axis vector (Cartographer
 * convention). The result direction is the rotation axis; its Euclidean length
 * is the rotation angle in radians. The quaternion sign is flipped if \(w < 0\)
 * so the angle stays in a consistent half-space.
 * @tparam T Scalar type.
 * @param[in] quaternion Input rotation (need not be normalized; it is
 *                       normalized internally).
 * @return 3-vector \(\phi\) such that \(\mathrm{Exp}([\phi]_\times)\) matches
 *         the quaternion (within numerical tolerance).
 */
template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<T>& quaternion) {
  Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
  if (normalized_quaternion.w() < 0.) {
    normalized_quaternion.coeffs() *= T(-1.);
  }
  const T angle = T(2.) * std::atan2(normalized_quaternion.vec().norm(),
                                     normalized_quaternion.w());
  constexpr double kCutoffAngle = 1e-7;
  const T scale = angle < kCutoffAngle ? T(2.) : angle / std::sin(angle / 2.);
  return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                scale * normalized_quaternion.y(),
                                scale * normalized_quaternion.z());
}

/**
 * @brief Convert an angle-axis vector to a unit quaternion.
 * @tparam T Scalar type.
 * @param[in] angle_axis 3-vector whose direction is the axis and length is the
 *                       angle in radians.
 * @return Normalized quaternion representing the same rotation.
 */
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) {
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = std::sin(norm / 2.) / norm;
    w = std::cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

/**
 * @brief Project a 3D pose to the horizontal plane.
 * @tparam T Scalar type.
 * @param[in] transform Input 3D pose.
 * @return @ref Rigid2 with \((x, y)\) from the translation and yaw from
 *         @ref GetYaw; \(z\) and roll/pitch are discarded.
 */
template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform) {
  return Rigid2<T>(transform.translation().template head<2>(),
                   GetYaw(transform));
}

/**
 * @brief Embed a planar pose into 3D with \(z = 0\) and rotation about +Z.
 * @tparam T Scalar type.
 * @param[in] transform Planar pose.
 * @return @ref Rigid3 with translation \((x, y, 0)\) and yaw-only rotation.
 */
template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) {
  return Rigid3<T>(
      {transform.translation().x(), transform.translation().y(), T(0)},
      Eigen::AngleAxis<T>(transform.rotation().angle(),
                          Eigen::Matrix<T, 3, 1>::UnitZ()));
}

/**
 * @brief Convert an Eigen double isometry to @ref Rigid3d.
 * @param[in] isometry Eigen `Isometry3d` pose.
 * @return Equivalent @ref Rigid3d (translation + quaternion from linear part).
 */
inline Rigid3d ToRigid3(const Eigen::Isometry3d& isometry) {
  return Rigid3d(isometry.translation(), Eigen::Quaterniond(isometry.linear()));
}

/**
 * @brief Convert @ref Rigid3d to an Eigen double isometry.
 * @param[in] rigid Input rigid pose.
 * @return Equivalent `Eigen::Isometry3d`.
 */
inline Eigen::Isometry3d ToIsometry(const Rigid3d& rigid) {
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = rigid.translation();
  isometry.linear() = rigid.rotation().toRotationMatrix();
  return isometry;
}

/**
 * @brief Convert an Eigen float isometry to @ref Rigid3f.
 * @param[in] isometry Eigen `Isometry3f` pose.
 * @return Equivalent @ref Rigid3f.
 */
inline Rigid3f ToRigid3(const Eigen::Isometry3f& isometry) {
  return Rigid3f(isometry.translation(), Eigen::Quaternionf(isometry.linear()));
}

/**
 * @brief Convert @ref Rigid3f to an Eigen float isometry.
 * @param[in] rigid Input rigid pose.
 * @return Equivalent `Eigen::Isometry3f`.
 */
inline Eigen::Isometry3f ToIsometry(const Rigid3f& rigid) {
  Eigen::Isometry3f isometry = Eigen::Isometry3f::Identity();
  isometry.translation() = rigid.translation();
  isometry.linear() = rigid.rotation().toRotationMatrix();
  return isometry;
}

/**
 * @brief Build @ref Rigid3d from translation and URDF-style ZYX Euler angles.
 * @param[in] x     Translation X (metres).
 * @param[in] y     Translation Y (metres).
 * @param[in] z     Translation Z (metres).
 * @param[in] roll  Rotation about X (radians).
 * @param[in] pitch Rotation about Y (radians).
 * @param[in] yaw   Rotation about Z (radians).
 * @return Composed rigid pose.
 */
inline Rigid3d FromXyzRpy(double x, double y, double z, double roll,
                          double pitch, double yaw) {
  return Rigid3d(Eigen::Vector3d(x, y, z), RollPitchYaw(roll, pitch, yaw));
}

/**
 * @brief Build a planar @ref Rigid3d (zero roll and pitch).
 * @param[in] x   Translation X (metres).
 * @param[in] y   Translation Y (metres).
 * @param[in] z   Translation Z (metres).
 * @param[in] yaw Heading about +Z (radians).
 * @return Rigid pose with roll = pitch = 0.
 */
inline Rigid3d FromXyzYaw(double x, double y, double z, double yaw) {
  return FromXyzRpy(x, y, z, 0.0, 0.0, yaw);
}

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_TRANSFORM_HPP_
