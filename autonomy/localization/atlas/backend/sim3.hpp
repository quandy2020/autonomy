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
 * @file sim3.hpp
 * @brief Similarity transform Sim3 (ORB-SLAM3 g2o::Sim3 counterpart, Ceres-friendly).
 *
 * Defines \(p' = s R p + t\); used for loop correction, map merging, and OptimizeSim3.
 * Differs from SE3 by the extra scale \(s\) (often fixed to \(s=1\) for stereo/RGB-D).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_SIM3_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_SIM3_HPP_

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

/**
 * @struct autonomy::localization::atlas::backend::Sim3
 * @brief Similarity transform: \(p' = \mathrm{scale} \cdot R \cdot p + t\).
 *
 * Members are stored as scale + rotation + translation (not a raw 4x4 matrix)
 * so they can be optimized as separate parameters.
 * Composition: `Compose(other)` means `(*this) ∘ other` (apply `other.Map` first,
 * then `Map`).
 */
struct Sim3 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double scale = 1.0;              ///< Scale \(s\) (optimizable in mono loop; often 1 for stereo)
    Mat33 rotation = Mat33::Identity();  ///< Rotation \(R \in SO(3)\)
    Vec3 translation = Vec3::Zero();     ///< Translation \(t\)

    /**
     * @brief Map point \(p\) to \(s R p + t\).
     * @param point Input 3D point (typically in a camera frame).
     * @return Transformed 3D point.
     */
    Vec3 Map(const Vec3& point) const {
        return scale * (rotation * point) + translation;
    }

    /**
     * @brief Inverse similarity: \(s'=1/s\), \(R'=R^\top\), \(t'=-s' R' t\).
     * @return Sim3 satisfying `Inverse().Map(Map(p)) ≈ p`.
     */
    Sim3 Inverse() const {
        Sim3 inv;
        inv.scale = 1.0 / scale;
        inv.rotation = rotation.transpose();
        inv.translation = -inv.scale * (inv.rotation * translation);
        return inv;
    }

    /**
     * @brief Embed as a homogeneous 4x4 matrix \([sR\ t;\ 0\ 1]\).
     * @return Matrix suitable for SE3/Eigen transform chains.
     */
    Eigen::Matrix4d Matrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = scale * rotation;
        T.block<3, 1>(0, 3) = translation;
        return T;
    }

    /**
     * @brief Unit similarity (\(s=1,\ R=I,\ t=0\)).
     * @return Identity Sim3.
     */
    static Sim3 Identity() { return Sim3{}; }

    /**
     * @brief Build from SE3 (scale fixed to 1).
     * @param pose Rigid pose (rotation and translation).
     * @return Corresponding Sim3.
     */
    static Sim3 FromSe3(const SE3& pose) {
        Sim3 s;
        s.scale = 1.0;
        s.rotation = pose.rotation();
        s.translation = pose.translation();
        return s;
    }

    /**
     * @brief Compose: `(*this) ∘ other`, i.e. `Map(other.Map(p))`.
     * @param other Right-hand (applied first) similarity.
     * @return Composed Sim3.
     */
    Sim3 Compose(const Sim3& other) const {
        Sim3 out;
        out.scale = scale * other.scale;
        out.rotation = rotation * other.rotation;
        out.translation =
            scale * (rotation * other.translation) + translation;
        return out;
    }
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_SIM3_HPP_
