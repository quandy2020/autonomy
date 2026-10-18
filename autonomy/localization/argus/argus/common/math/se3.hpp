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
 * @file se3.hpp
 * @brief SE(3) facade over `transform::Rigid3d` (FAST-LIVO2 accessor names).
 */

#ifndef ARGUS_COMMON_MATH_SE3_HPP_
#define ARGUS_COMMON_MATH_SE3_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/common/math/transform/transform.hpp"

namespace argus {
namespace math {

/**
 * @brief Rigid transform \(T = [R|t]\): \(p_w = R p_b + t\) (world ← body).
 *
 * Storage is `transform::Rigid3d`. Method names match historical FAST-LIVO2
 * `Sophus::SE3` call sites (`rotation_matrix`, `translation`, `inverse`).
 */
class SE3
{
public:
    SE3() = default;

    /**
     * @brief Construct from rotation matrix and translation.
     * @param R 3×3 rotation (world ← body).
     * @param t Translation vector.
     */
    SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
        : T_(t, Eigen::Quaterniond(R)) {}

    /** @param T Underlying rigid transform. */
    explicit SE3(const transform::Rigid3d& T) : T_(T) {}

    /** @param T Eigen isometry converted via @ref transform::ToRigid3. */
    explicit SE3(const Eigen::Isometry3d& T) : T_(transform::ToRigid3(T)) {}

    /** @return 3×3 rotation matrix. */
    Eigen::Matrix3d rotation_matrix() const {
        return T_.rotation().toRotationMatrix();
    }
    /** @return Same as @ref rotation_matrix (FAST-LIVO2 alias). */
    Eigen::Matrix3d rotationMatrix() const {
        return rotation_matrix();
    }
    /** @return Translation vector. */
    Eigen::Vector3d translation() const {
        return T_.translation();
    }

    /** @return Const reference to internal @ref transform::Rigid3d. */
    const transform::Rigid3d& rigid() const {
        return T_;
    }

    /** @return Inverse rigid transform. */
    SE3 inverse() const {
        return SE3(T_.inverse());
    }

    /**
     * @brief Transform a 3D point: p' = R p + t.
     * @param p Point in body frame.
     * @return Point in world frame.
     */
    Eigen::Vector3d operator*(const Eigen::Vector3d& p) const {
        return T_ * p;
    }

    /** @param other Right-hand transform. @return Composed transform. */
    SE3 operator*(const SE3& other) const {
        return SE3(T_ * other.T_);
    }

    /** @return Equivalent Eigen::Isometry3d. */
    Eigen::Isometry3d toIsometry() const {
        return transform::ToIsometry(T_);
    }

private:
    transform::Rigid3d T_ = transform::Rigid3d::Identity();
};

/** @param T SE3 facade. @return Underlying @ref transform::Rigid3d. */
inline transform::Rigid3d ToRigid3(const SE3& T) {
    return T.rigid();
}
/** @param rigid Cartographer-style pose. @return SE3 facade. */
inline SE3 ToSE3(const transform::Rigid3d& rigid) {
    return SE3(rigid);
}

}  // namespace math

namespace transform {

/** @param T SE3 in math namespace. @return Rigid3d in transform namespace. */
inline Rigid3d ToRigid3(const math::SE3& T) {
    return math::ToRigid3(T);
}
/** @param rigid Rigid3d. @return SE3 in math namespace. */
inline math::SE3 ToSE3(const Rigid3d& rigid) {
    return math::ToSE3(rigid);
}

}  // namespace transform
}  // namespace argus

#endif  // ARGUS_COMMON_MATH_SE3_HPP_
