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
 *
 * Bearing-vector PnP RANSAC (ORB-SLAM3 MLPnPsolver role; Ceres-free seed).
 */

/**
 * @file mlpnp_solver.hpp
 * @brief Bearing-vector MLPnP + RANSAC for relocalization pose estimation.
 *
 * Role aligned with ORB-SLAM3 MLPnPsolver: analytic MLPnP (bearing nullspace,
 * planar/non-planar SVD init, Gauss–Newton + Rodrigues Jacobian), RANSAC on
 * 6-point minimal samples.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_SOLVE_MLPNP_SOLVER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_SOLVE_MLPNP_SOLVER_HPP_

#include <memory>
#include <vector>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace frontend {

/**
 * @class autonomy::localization::atlas::frontend::MlpnpSolver
 * @brief Recover relocalization pose from 3D–bearing matches (Urban MLPnP / ORB role).
 *
 * Constructor extracts bearings and world points from Frame + matched MapPoints;
 * `Find` outputs Tcw and an inlier mask. Mask length equals internal correspondences;
 * see `indices()`.
 *
 * @note Typically constructed temporarily on the Tracking-thread Relocalization path.
 */
class MlpnpSolver {
public:
    /**
     * @brief Build solver from current frame and MapPoint match list.
     * @param frame Current frame (intrinsics, features, scale info).
     * @param matches MapPoints aligned with frame features (nullptr = no match).
     */
    MlpnpSolver(const tracking::Frame& frame,
                const std::vector<std::shared_ptr<MapPoint>>& matches);

    /**
     * @brief Configure RANSAC parameters.
     * @param probability Desired success probability (adaptive iteration count).
     * @param min_inliers Minimum inlier count.
     * @param max_iterations Maximum iterations.
     * @param reproj_th Reprojection χ² threshold (~2DOF 95%, default 5.991).
     */
    void SetRansacParameters(double probability = 0.99, int min_inliers = 8,
                             int max_iterations = 300, float reproj_th = 5.991f);

    /**
     * @brief Estimate camera pose Tcw via RANSAC + MLPnP.
     * @param[out] Tcw World → camera pose.
     * @param[out] inliers Inlier mask; length equals internal correspondences (indices()).
     * @param[out] num_inliers Inlier count.
     * @return true on success; false if samples insufficient or estimation failed.
     */
    bool Find(SE3* Tcw, std::vector<bool>* inliers, int* num_inliers);

    /**
     * @brief Map from internal correspondence index to frame feature index.
     * @return Frame feature index per correspondence.
     */
    const std::vector<int>& indices() const { return indices_; }

private:
    /**
     * @brief Count inliers for a candidate pose.
     * @param Tcw Candidate pose.
     * @param[out] inliers Inlier mask.
     * @return Inlier count.
     */
    int CheckInliers(const SE3& Tcw, std::vector<bool>* inliers) const;

    std::vector<Vec3> bearings_;       ///< Unit bearing vectors
    std::vector<Vec3> points_world_;   ///< Corresponding world points
    std::vector<int> indices_;         ///< Frame feature indices

    double probability_ = 0.99;
    int min_inliers_ = 8;
    int max_iterations_ = 300;
    float reproj_th_ = 5.991f;  ///< χ² 2DOF ~95%
    float fx_ = 1.f;
    float fy_ = 1.f;
    std::vector<float> sigma2_;
    std::vector<float> max_error_;
};

}  // namespace frontend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_SOLVE_MLPNP_SOLVER_HPP_
