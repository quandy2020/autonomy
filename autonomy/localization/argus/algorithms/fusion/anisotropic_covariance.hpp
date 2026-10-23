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
 * @file anisotropic_covariance.hpp
 * @brief Rank-1 covariance inflation along weak observability directions.
 */

#ifndef ARGUS_FUSION_ANISOTROPIC_COVARIANCE_HPP_
#define ARGUS_FUSION_ANISOTROPIC_COVARIANCE_HPP_

#include <Eigen/Core>

namespace argus {
namespace fusion {

/** @brief Utilities to inflate state covariance along degenerate directions. */
class AnisotropicCovariance
{
public:
    /**
     * @brief Inflate the leading 3×3 (position) block: P ← P + α u uᵀ.
     * @tparam Derived Eigen matrix expression with at least 3×3 top-left block.
     * @param[in,out] P State covariance (modified in place when @p alpha > 0).
     * @param weak_direction Direction of poor observability (normalized
     *        internally).
     * @param alpha Non-negative inflation strength α.
     */
    template <typename Derived>
    static void InflatePositionBlock(Eigen::MatrixBase<Derived>& P,
                                     const Eigen::Vector3d& weak_direction,
                                     double alpha) {
        if (alpha <= 0.0 || P.rows() < 3 || P.cols() < 3) {
            return;
        }
        const Eigen::Vector3d u = weak_direction.normalized();
        P.derived().template topLeftCorner<3, 3>() +=
            alpha * (u * u.transpose());
    }

    /**
     * @brief Full-matrix wrapper around @ref InflatePositionBlock.
     * @param[in,out] P Covariance matrix (no-op if null or @p alpha ≤ 0).
     * @param weak_direction Weak observability direction.
     * @param alpha Inflation strength α.
     */
    static void Inflate(Eigen::MatrixXd* P,
                        const Eigen::Vector3d& weak_direction, double alpha);
};

}  // namespace fusion
}  // namespace argus

#endif  // ARGUS_FUSION_ANISOTROPIC_COVARIANCE_HPP_
