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
 * @brief Rigid2 / Rigid3 (ported from Cartographer
 * `transform/rigid_transform`).
 *
 * No Lua / proto dependencies. Argus uses this as the system pose type.
 */

#ifndef ARGUS_COMMON_MATH_TRANSFORM_RIGID_TRANSFORM_HPP_
#define ARGUS_COMMON_MATH_TRANSFORM_RIGID_TRANSFORM_HPP_

#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace transform {

/**
 * @brief Wrap angle difference into \([-π, π]\).
 * @param difference Raw angle difference (radians).
 * @return Wrapped difference.
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

/** @brief 2D rigid transform: translation + planar rotation. */
template <typename FloatType>
class Rigid2
{
public:
    using Vector = Eigen::Matrix<FloatType, 2, 1>;
    using Rotation2D = Eigen::Rotation2D<FloatType>;

    Rigid2()
        : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
    Rigid2(const Vector& translation, const Rotation2D& rotation)
        : translation_(translation), rotation_(rotation) {}
    Rigid2(const Vector& translation, const double rotation)
        : translation_(translation), rotation_(rotation) {}

    /** @param rotation Angle in radians. @return Pure rotation transform. */
    static Rigid2 Rotation(const double rotation) {
        return Rigid2(Vector::Zero(), rotation);
    }
    /** @param rotation 2D rotation object. @return Pure rotation transform. */
    static Rigid2 Rotation(const Rotation2D& rotation) {
        return Rigid2(Vector::Zero(), rotation);
    }
    /** @param vector Translation. @return Pure translation transform. */
    static Rigid2 Translation(const Vector& vector) {
        return Rigid2(vector, Rotation2D::Identity());
    }
    /** @return Identity transform. */
    static Rigid2<FloatType> Identity() {
        return Rigid2<FloatType>();
    }

    /** @return Same pose with a different scalar type. */
    template <typename OtherType>
    Rigid2<OtherType> cast() const {
        return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                                 rotation_.template cast<OtherType>());
    }

    /** @return Translation vector. */
    const Vector& translation() const {
        return translation_;
    }
    /** @return Planar rotation. */
    Rotation2D rotation() const {
        return rotation_;
    }

    /** @return Rotation angle wrapped to \([-π, π]\). */
    double normalized_angle() const {
        return NormalizeAngleDifference(rotation().angle());
    }

    /** @return Inverse rigid transform. */
    Rigid2 inverse() const {
        const Rotation2D rotation = rotation_.inverse();
        const Vector translation = -(rotation * translation_);
        return Rigid2(translation, rotation);
    }

    /** @return Human-readable debug representation. */
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

/** @return Composition lhs ∘ rhs (apply rhs first). */
template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
    return Rigid2<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
    return rigid.rotation() * point + rigid.translation();
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Rigid2<T>& rigid) {
    os << rigid.DebugString();
    return os;
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

/** @brief 3D rigid transform: translation + unit quaternion rotation. */
template <typename FloatType>
class Rigid3
{
public:
    using Vector = Eigen::Matrix<FloatType, 3, 1>;
    using Quaternion = Eigen::Quaternion<FloatType>;
    using AngleAxis = Eigen::AngleAxis<FloatType>;

    Rigid3()
        : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
    Rigid3(const Vector& translation, const Quaternion& rotation)
        : translation_(translation), rotation_(rotation) {}
    Rigid3(const Vector& translation, const AngleAxis& rotation)
        : translation_(translation), rotation_(rotation) {}

    /** @param angle_axis Axis-angle rotation. @return Pure rotation transform.
     */
    static Rigid3 Rotation(const AngleAxis& angle_axis) {
        return Rigid3(Vector::Zero(), Quaternion(angle_axis));
    }
    /** @param rotation Unit quaternion. @return Pure rotation transform. */
    static Rigid3 Rotation(const Quaternion& rotation) {
        return Rigid3(Vector::Zero(), rotation);
    }
    /** @param vector Translation. @return Pure translation transform. */
    static Rigid3 Translation(const Vector& vector) {
        return Rigid3(vector, Quaternion::Identity());
    }
    /**
     * @brief Build from quaternion (w,x,y,z) and translation arrays.
     * @param rotation Four quaternion components [w,x,y,z].
     * @param translation Three translation components [x,y,z].
     * @return Rigid3 pose.
     */
    static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation,
                             const std::array<FloatType, 3>& translation) {
        return Rigid3(Eigen::Map<const Vector>(translation.data()),
                      Eigen::Quaternion<FloatType>(rotation[0], rotation[1],
                                                   rotation[2], rotation[3]));
    }
    /** @return Identity transform. */
    static Rigid3<FloatType> Identity() {
        return Rigid3<FloatType>();
    }

    /** @return Same pose with a different scalar type. */
    template <typename OtherType>
    Rigid3<OtherType> cast() const {
        return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                                 rotation_.template cast<OtherType>());
    }

    /** @return Translation vector. */
    const Vector& translation() const {
        return translation_;
    }
    /** @return Unit quaternion rotation. */
    const Quaternion& rotation() const {
        return rotation_;
    }

    /** @return Inverse rigid transform. */
    Rigid3 inverse() const {
        const Quaternion rotation = rotation_.conjugate();
        const Vector translation = -(rotation * translation_);
        return Rigid3(translation, rotation);
    }

    /** @return Human-readable debug representation. */
    std::string DebugString() const {
        char buf[256];
        std::snprintf(
            buf, sizeof(buf), "{ t: [%f, %f, %f], q: [%f, %f, %f, %f] }",
            translation().x(), translation().y(), translation().z(),
            rotation().w(), rotation().x(), rotation().y(), rotation().z());
        return std::string(buf);
    }

    /** @return False if translation or quaternion norm is invalid. */
    bool IsValid() const {
        return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
               !std::isnan(translation_.z()) &&
               std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
    }

private:
    Vector translation_;
    Quaternion rotation_;
};

/** @return Composition lhs ∘ rhs (apply rhs first). */
template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
    return Rigid3<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
    return rigid.rotation() * point + rigid.translation();
}

template <typename T>
std::ostream& operator<<(std::ostream& os, const Rigid3<T>& rigid) {
    os << rigid.DebugString();
    return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

/**
 * @brief URDF ZYX: yaw × pitch × roll → unit quaternion.
 * @param roll Rotation about fixed X (rad).
 * @param pitch Rotation about fixed Y (rad).
 * @param yaw Rotation about fixed Z (rad).
 * @return Unit quaternion.
 */
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_MATH_TRANSFORM_RIGID_TRANSFORM_HPP_
