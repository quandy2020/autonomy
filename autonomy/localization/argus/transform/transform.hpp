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
 * @brief Rigid transform helpers + Eigen / SE3 bridges (Cartographer-style).
 */

#ifndef ARGUS_COMMON_MATH_TRANSFORM_TRANSFORM_HPP_
#define ARGUS_COMMON_MATH_TRANSFORM_TRANSFORM_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "argus/common/math/transform/rigid_transform.hpp"

namespace argus {
namespace transform {

/** @param transform Rigid3 pose. @return Rotation angle magnitude (radians). */
template <typename FloatType>
FloatType GetAngle(const Rigid3<FloatType>& transform) {
    return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                     std::abs(transform.rotation().w()));
}

/** @param rotation Unit quaternion. @return Yaw about +Z (radians). */
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) {
    const Eigen::Matrix<T, 3, 1> direction =
        rotation * Eigen::Matrix<T, 3, 1>::UnitX();
    return std::atan2(direction.y(), direction.x());
}

/** @param transform Rigid3 pose. @return Yaw about +Z (radians). */
template <typename T>
T GetYaw(const Rigid3<T>& transform) {
    return GetYaw(transform.rotation());
}

/**
 * @brief Quaternion to angle-axis vector (Cartographer convention).
 * @param quaternion Input rotation.
 * @return 3-vector whose direction is axis and length is angle.
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
 * @brief Angle-axis vector to unit quaternion.
 * @param angle_axis 3-vector (axis × angle).
 * @return Normalized quaternion.
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

/** @param transform 3D pose. @return Planar (x, y, yaw) projection. */
template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform) {
    return Rigid2<T>(transform.translation().template head<2>(),
                     GetYaw(transform));
}

/** @param transform Planar pose. @return 3D pose with z = 0 and rotation about
 * Z. */
template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) {
    return Rigid3<T>(
        {transform.translation().x(), transform.translation().y(), T(0)},
        Eigen::AngleAxis<T>(transform.rotation().angle(),
                            Eigen::Matrix<T, 3, 1>::UnitZ()));
}

/** @param T Eigen isometry. @return Equivalent Rigid3d. */
inline Rigid3d ToRigid3(const Eigen::Isometry3d& T) {
    return Rigid3d(T.translation(), Eigen::Quaterniond(T.linear()));
}

/** @param rigid Rigid3d pose. @return Eigen::Isometry3d. */
inline Eigen::Isometry3d ToIsometry(const Rigid3d& rigid) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = rigid.translation();
    T.linear() = rigid.rotation().toRotationMatrix();
    return T;
}

/** @param T Float isometry. @return Equivalent Rigid3f. */
inline Rigid3f ToRigid3(const Eigen::Isometry3f& T) {
    return Rigid3f(T.translation(), Eigen::Quaternionf(T.linear()));
}

/** @param rigid Rigid3f pose. @return Eigen::Isometry3f. */
inline Eigen::Isometry3f ToIsometry(const Rigid3f& rigid) {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation() = rigid.translation();
    T.linear() = rigid.rotation().toRotationMatrix();
    return T;
}

/**
 * @brief Build Rigid3d from translation and URDF ZYX Euler angles.
 * @param x,y,z Translation (m).
 * @param roll,pitch,yaw Fixed-frame Euler (rad).
 * @return Rigid3d pose.
 */
inline Rigid3d FromXyzRpy(double x, double y, double z, double roll,
                          double pitch, double yaw) {
    return Rigid3d(Eigen::Vector3d(x, y, z), RollPitchYaw(roll, pitch, yaw));
}

/**
 * @brief Planar pose with zero roll and pitch.
 * @param x,y,z Translation (m).
 * @param yaw Heading about +Z (rad).
 * @return Rigid3d pose.
 */
inline Rigid3d FromXyzYaw(double x, double y, double z, double yaw) {
    return FromXyzRpy(x, y, z, 0.0, 0.0, yaw);
}

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_MATH_TRANSFORM_TRANSFORM_HPP_
