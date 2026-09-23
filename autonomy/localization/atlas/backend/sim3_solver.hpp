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
 * @file sim3_solver.hpp
 * @brief Sim3 estimator from 3D–3D matches (Horn 1987 + RANSAC, ORB-SLAM3 aligned).
 *
 * Input is MapPoint correspondences between two keyframes; output is a
 * camera-frame similarity for LoopClosing coarse estimate and subsequent
 * `Optimizer::OptimizeSim3` refinement.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_SIM3_SOLVER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_SIM3_SOLVER_HPP_

#include <memory>
#include <vector>

#include "autonomy/localization/atlas/backend/sim3.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

/**
 * @class autonomy::localization::atlas::backend::Sim3Solver
 * @brief Estimate Sim3 from 3D–3D matches between two keyframes (cam1 ← cam2).
 *
 * Flow: constructor moves matches into each camera frame → `Find` runs RANSAC
 * (Horn with at least three points) → inliers filtered by reprojection/distance
 * thresholds. With `fix_scale=true`, force \(s=1\) (stereo/RGB-D).
 */
class Sim3Solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Build solver from KF1/KF2 and matches12.
     * @param keyframe1 Reference keyframe (observation side).
     * @param keyframe2 Matched keyframe.
     * @param matches12 KF2 MapPoints aligned with KF1 feature indices (nullptr = no match).
     * @param fix_scale If true, scale is fixed to 1.
     */
    Sim3Solver(const std::shared_ptr<KeyFrame>& keyframe1,
               const std::shared_ptr<KeyFrame>& keyframe2,
               const std::vector<std::shared_ptr<MapPoint>>& matches12,
               bool fix_scale = true);

    /**
     * @brief Set RANSAC parameters.
     * @param probability Desired probability of drawing an all-inlier sample at least once.
     * @param min_inliers Minimum inliers to accept a solution.
     * @param max_iterations Maximum iterations.
     */
    void SetRansacParameters(double probability = 0.99, int min_inliers = 6,
                             int max_iterations = 300);

    /**
     * @brief Run RANSAC; return best Sim3 (identity on failure).
     * @param[out] inliers12 Inlier mask same length as matches12 (may be null).
     * @param[out] num_inliers Inlier count (may be null).
     * @return Best estimated Sim3.
     */
    Sim3 Find(std::vector<bool>* inliers12, int* num_inliers);

    /** @brief Most recent successfully estimated Sim3. */
    Sim3 estimated() const { return best_sim3_; }
    /** @brief Scale from the most recent estimate. */
    double estimated_scale() const { return best_sim3_.scale; }

private:
    /**
     * @brief Compute point-set centroid and de-mean.
     * @param P Column-major 3xN point matrix.
     * @param[out] Pr De-meaned points.
     * @param[out] centroid Centroid.
     */
    void ComputeCentroid(const Eigen::Matrix3d& P, Eigen::Matrix3d* Pr,
                         Vec3* centroid) const;
    /**
     * @brief Horn closed-form: current Sim3 from de-meaned point sets.
     * @param P1 Camera-1 points (3x3 minimal-sample columns).
     * @param P2 Camera-2 points.
     */
    void ComputeSim3(const Eigen::Matrix3d& P1, const Eigen::Matrix3d& P2);
    /** @brief Count inliers under current Sim3 and update `current_*`. */
    void CheckInliers();

    std::shared_ptr<KeyFrame> keyframe1_;  ///< Reference keyframe
    std::shared_ptr<KeyFrame> keyframe2_;  ///< Matched keyframe
    std::vector<Vec3> points_camera1_;     ///< 3D points in KF1 camera frame
    std::vector<Vec3> points_camera2_;     ///< Corresponding points in KF2 camera frame
    std::vector<size_t> indices1_;         ///< Valid match indices on KF1
    int num_matches_ = 0;                  ///< Valid 3D–3D match count
    bool fix_scale_ = true;                ///< Whether scale is fixed

    Sim3 current_sim3_;                    ///< Current RANSAC hypothesis
    std::vector<bool> current_inliers_;    ///< Inliers for current hypothesis
    int current_inliers_count_ = 0;        ///< Current inlier count

    Sim3 best_sim3_;                       ///< Best Sim3 so far
    std::vector<bool> best_inliers_;       ///< Best inlier mask
    int best_inliers_count_ = 0;           ///< Best inlier count

    double ransac_probability_ = 0.99;     ///< RANSAC success probability
    int ransac_min_inliers_ = 6;           ///< Minimum inliers
    int ransac_max_iterations_ = 300;      ///< Max iterations
    double inlier_threshold_ = 0.05;       ///< Camera-frame distance threshold (m)
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_SIM3_SOLVER_HPP_
