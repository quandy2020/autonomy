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
 * @file so3.hpp
 * @brief SO(3) Rodrigues Exp / Log and ZYX Euler helpers (FAST-LIVO2 numerics).
 *
 * Ported from `atlas/livo/include/utils/so3_math.h`. Prefer these formulas in
 * Argus / LIVO hot paths for bit-compatible propagation with the original
 * stack. Related quaternion-based helpers live in
 * `autonomy/common/math/so3.hpp`.
 */

#ifndef ARGUS_COMMON_MATH_SO3_HPP_
#define ARGUS_COMMON_MATH_SO3_HPP_

#include <cmath>

#include "Eigen/Core"

namespace argus {
namespace math {

/**
 * @brief Comma-init list for `Matrix3 << SkewSymmetricList(v)`.
 *
 * Expands to nine scalars: 0, -z, y, z, 0, -x, -y, x, 0.
 */
#define ARGUS_SKEW_SYM_LIST(v) \
    0.0, -(v)[2], (v)[1], (v)[2], 0.0, -(v)[0], -(v)[1], (v)[0], 0.0

/** @brief Legacy alias used throughout FAST-LIVO2 call sites. */
#ifndef SKEW_SYM_MATRX
#define SKEW_SYM_MATRX(v) ARGUS_SKEW_SYM_LIST(v)
#endif

/**
 * @brief Skew-symmetric matrix [v]_× ∈ so(3).
 * @tparam Derived Eigen 3-vector expression.
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
    const Eigen::MatrixBase<Derived>& vector) {
    using Scalar = typename Derived::Scalar;
    Eigen::Matrix<Scalar, 3, 3> skew;
    skew << Scalar(0), -vector(2), vector(1), vector(2), Scalar(0), -vector(0),
        -vector(1), vector(0), Scalar(0);
    return skew;
}

/**
 * @brief SO(3) exponential map (Rodrigues): φ → Exp([φ]_×).
 * @param angle Angle-axis vector (radians · axis).
 */
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& angle) {
    const T angle_norm = angle.norm();
    const Eigen::Matrix<T, 3, 3> eye = Eigen::Matrix<T, 3, 3>::Identity();
    if (angle_norm > T(1e-7)) {
        const Eigen::Matrix<T, 3, 1> axis = angle / angle_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(axis);
        return eye + std::sin(angle_norm) * K +
               (T(1) - std::cos(angle_norm)) * K * K;
    }
    return eye;
}

/**
 * @brief Rvalue overload retained for historical FAST-LIVO2 call sites.
 */
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(Eigen::Matrix<T, 3, 1>&& angle) {
    return Exp(static_cast<const Eigen::Matrix<T, 3, 1>&>(angle));
}

/**
 * @brief Integrated exponential: Exp(ω · dt).
 * @param angular_velocity Body angular velocity ω.
 * @param dt Time step (may be negative).
 */
template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& angular_velocity,
                           const Ts& dt) {
    const T omega_norm = angular_velocity.norm();
    const Eigen::Matrix<T, 3, 3> eye = Eigen::Matrix<T, 3, 3>::Identity();
    if (omega_norm > T(1e-7)) {
        const Eigen::Matrix<T, 3, 1> axis = angular_velocity / omega_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(axis);
        const T angle = omega_norm * static_cast<T>(dt);
        return eye + std::sin(angle) * K + (T(1) - std::cos(angle)) * K * K;
    }
    return eye;
}

/**
 * @brief Component-wise Exp(v1, v2, v3).
 */
template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
    const T norm = std::sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    const Eigen::Matrix<T, 3, 3> eye = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > T(1e-5)) {
        const T axis[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(axis);
        return eye + std::sin(norm) * K + (T(1) - std::cos(norm)) * K * K;
    }
    return eye;
}

/**
 * @brief SO(3) logarithm: R → φ such that Exp([φ]_×) = R.
 */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& rotation) {
    const T theta = (rotation.trace() > T(3) - T(1e-6))
                        ? T(0)
                        : std::acos(T(0.5) * (rotation.trace() - T(1)));
    const Eigen::Matrix<T, 3, 1> K(rotation(2, 1) - rotation(1, 2),
                                   rotation(0, 2) - rotation(2, 0),
                                   rotation(1, 0) - rotation(0, 1));
    return (std::abs(theta) < T(0.001))
               ? (T(0.5) * K)
               : (T(0.5) * theta / std::sin(theta) * K);
}

/**
 * @brief ZYX Euler angles (roll, pitch, yaw) from a rotation matrix.
 *
 * Same convention as historical FAST-LIVO2 `RotMtoEuler`.
 */
template <typename T>
Eigen::Matrix<T, 3, 1> RotationMatrixToRollPitchYaw(
    const Eigen::Matrix<T, 3, 3>& rotation) {
    const T sine_yaw = std::sqrt(rotation(0, 0) * rotation(0, 0) +
                                 rotation(1, 0) * rotation(1, 0));
    const bool singular = sine_yaw < T(1e-6);
    T roll;
    T pitch;
    T yaw;
    if (!singular) {
        roll = std::atan2(rotation(2, 1), rotation(2, 2));
        pitch = std::atan2(-rotation(2, 0), sine_yaw);
        yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    } else {
        roll = std::atan2(-rotation(1, 2), rotation(1, 1));
        pitch = std::atan2(-rotation(2, 0), sine_yaw);
        yaw = T(0);
    }
    return Eigen::Matrix<T, 3, 1>(roll, pitch, yaw);
}

/** @brief Legacy name used by FAST-LIVO2. */
template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rotation) {
    return RotationMatrixToRollPitchYaw(rotation);
}

}  // namespace math
}  // namespace argus

#endif  // ARGUS_COMMON_MATH_SO3_HPP_
