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
 * @file optimizer.hpp
 * @brief Ceres implementation of the ORB-SLAM3 Optimizer API (replaces g2o).
 *
 * Covers pose optimization, local/global BA, Essential Graph, Sim3 refinement,
 * inertial BA / initialization, etc. Parameter-block conventions are in each
 * CostFunctor: poses are usually `[aa(3), t(3)]` for \(T_{cw}\) or \(T_{wb}\).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_OPTIMIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_OPTIMIZER_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include "autonomy/localization/atlas/backend/cost_functions/lidar_cost_function.hpp"
#include "autonomy/localization/atlas/backend/sim3.hpp"
#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

/**
 * @class autonomy::localization::atlas::backend::Optimizer
 * @brief Static optimization entry points: visual BA / pose graph / VIO joint opt.
 *
 * All methods are static; callers own map locks and `stop_flag` coordination
 * (LocalMapping may interrupt).
 */
class Optimizer {
public:
    /**
     * @brief Optimize current-frame pose only (ORB PoseOptimization).
     * @param frame Frame with MapPoint matches; updates pose and outlier flags in place.
     * @return Number of visual inliers after optimization.
     */
    static int PoseOptimization(tracking::Frame* frame);

    /**
     * @brief Local BA around a keyframe neighborhood (ORB LocalBundleAdjustment subset).
     * @param keyframe Window-center keyframe.
     * @param map Owning map.
     * @param stop_flag If non-null, optimization may be aborted when set true externally.
     */
    static void LocalBundleAdjustment(const std::shared_ptr<KeyFrame>& keyframe,
                                      Map* map, bool* stop_flag = nullptr);

    /**
     * @brief Full-map BA (keyframes + map-point subset).
     * @param map Target map.
     * @param iterations Ceres max iterations.
     * @param stop_flag Optional abort flag.
     * @param gba_id Non-zero stores optimized poses for later spanning-tree
     *        propagation (ORB `nLoopKF`). Zero writes poses immediately.
     */
    static void GlobalBundleAdjustment(Map* map, int iterations = 5,
                                       bool* stop_flag = nullptr,
                                       uint64_t gba_id = 0);

    /**
     * @brief Essential Graph (SE3): fix loop keyframe, optimize remaining \(T_{cw}\).
     * @param map Map.
     * @param loop_keyframe Loop-matched keyframe (fixed).
     * @param current_keyframe Current keyframe.
     * @param loop_relative_pose_cw Loop relative \(T_{cw}\) (measurement edge).
     * @param pose_before Poses before loop correction. Relative edges use these;
     *        vertices start at the corrected `GetPose()`.
     */
    static void OptimizeEssentialGraph(
        Map* map, const std::shared_ptr<KeyFrame>& loop_keyframe,
        const std::shared_ptr<KeyFrame>& current_keyframe,
        const SE3& loop_relative_pose_cw,
        const std::unordered_map<KeyFrame*, SE3>* pose_before = nullptr);

    /**
     * @brief 4DoF Essential Graph (yaw + translation) for IMU-aligned maps.
     * @param map Map.
     * @param loop_keyframe Loop keyframe (fixed).
     * @param current_keyframe Current keyframe.
     * @param loop_relative_pose_cw Loop relative pose.
     * @param pose_before Poses before loop correction, same role as in SE3 graph.
     */
    static void OptimizeEssentialGraph4DoF(
        Map* map, const std::shared_ptr<KeyFrame>& loop_keyframe,
        const std::shared_ptr<KeyFrame>& current_keyframe,
        const SE3& loop_relative_pose_cw,
        const std::unordered_map<KeyFrame*, SE3>* pose_before = nullptr);

    /**
     * @brief Refine Sim3 between two keyframes (ORB OptimizeSim3, Ceres).
     * @param keyframe1 / keyframe2 Paired keyframes.
     * @param matches12 KF2 MapPoints aligned with KF1 indices (nullptr skips entry).
     * @param[in,out] sim3_12 Init/result: maps KF2 camera points into KF1 camera frame.
     * @param thr2 Reprojection chi-square threshold (pixel² scale).
     * @param fix_scale If true, fix scale (stereo/RGB-D).
     * @param all_points If true, allow matches with no MapPoint on KF1 (ORB bAllPoints).
     * @return Number of inlier correspondences after refinement.
     */
    static int OptimizeSim3(
        const std::shared_ptr<KeyFrame>& keyframe1,
        const std::shared_ptr<KeyFrame>& keyframe2,
        const std::vector<std::shared_ptr<MapPoint>>& matches12, Sim3* sim3_12,
        float thr2 = 10.f, bool fix_scale = true, bool all_points = false);

    /**
     * @brief Local inertial BA: free temporal window, fixed anchor and visual poses, χ² outlier rejection.
     * @note Falls back to `LocalBundleAdjustment` when there is no IMU chain.
     *        The keyframe before the window is held fixed. The inertial edge into
     *        that anchor is down-weighted (information × 1e-2).
     * @param keyframe Window center.
     * @param map Map.
     * @param stop_flag Optional abort.
     * @param large ORB `bLarge`: 25-frame window, 4 iterations, keep a rising cost.
     * @param reinit ORB `bRecInit`: Huber kernel on every inertial edge.
     */
    static void LocalInertialBA(const std::shared_ptr<KeyFrame>& keyframe,
                                Map* map, bool* stop_flag = nullptr,
                                bool large = false, bool reinit = false);

    /**
     * @brief Full inertial BA: visual reprojection, preintegration, and bias prior together.
     * @param map Map.
     * @param iterations Iteration count.
     * @param fix_local Whether to fix keyframes outside the local window.
     * @param stop_flag Optional abort.
     * @param init_shared_bias Whether to share bias init values.
     * @param prior_g / prior_a Gyro/accel bias prior strength.
     * @param gba_id Non-zero defers pose/point writes (ORB `nLoopKF`).
     */
    static void FullInertialBA(Map* map, int iterations = 100,
                               bool fix_local = false, bool* stop_flag = nullptr,
                               bool init_shared_bias = true,
                               float prior_g = 1e2f, float prior_a = 1e6f,
                               uint64_t gba_id = 0);

    /**
     * @brief Visual-inertial init: Ceres EdgeInertialGS (vel, shared bias, \(R_{wg}\), scale).
     * @param map Map (poses usually fixed; optimize velocity/bias/gravity/scale).
     * @param[in,out] Rwg Gravity-direction rotation.
     * @param[in,out] scale Scale.
     * @param[out] bg / ba Gyro/accel biases.
     * @param monocular Scale is optimizable when monocular.
     * @param prior_g / prior_a Bias priors.
     * @param fix_gravity_scale Hold \(R_{wg}\) and scale (bias-only refinement).
     * @param covariance Optional covariance of free inertial blocks
     *        (bias, then gravity direction, then scale when those are free).
     * @note Caller then runs ApplyScaledRotation / UpdateFrameIMU.
     */
    static void InertialOptimization(Map* map, Mat33* Rwg, double* scale,
                                     Vec3* bg, Vec3* ba, bool monocular,
                                     float prior_g = 1e2f,
                                     float prior_a = 1e6f,
                                     bool fix_gravity_scale = false,
                                     Eigen::MatrixXd* covariance = nullptr);

    /**
     * @brief Bias-only inertial refinement (ORB InertialOptimization(bg, ba)).
     *
     * Poses, gravity direction, and scale stay fixed. Velocities and one shared
     * bias are optimized.
     */
    static void InertialOptimization(Map* map, Vec3* bg, Vec3* ba,
                                     float prior_g = 1e2f,
                                     float prior_a = 1e6f);

    /**
     * @brief Welding BA after map merge (local window around matched keyframes).
     * @param current_keyframe Keyframe on the current map.
     * @param merge_keyframe Keyframe on the merged map.
     * @param map Active map after merge.
     */
    static void WeldingBundleAdjustment(
        const std::shared_ptr<KeyFrame>& current_keyframe,
        const std::shared_ptr<KeyFrame>& merge_keyframe, Map* map);

    /**
     * @brief Joint inertial welding BA on the current chain and the merge chain.
     *
     * Up to 6 keyframes behind the current frame and a short chain around the
     * merge frame (previous, merge, next) share one problem. The keyframe
     * before the merge chain is fixed. Extra observers are pose-only.
     * @param current_keyframe Current keyframe.
     * @param merge_keyframe Merge-side keyframe.
     * @param map Active map.
     */
    static void MergeInertialBA(const std::shared_ptr<KeyFrame>& current_keyframe,
                                const std::shared_ptr<KeyFrame>& merge_keyframe,
                                Map* map);

    /**
     * @brief Visual-inertial pose opt vs last keyframe (ORB PoseInertialOptimizationLastKeyFrame).
     * @param frame Current frame.
     * @param last_keyframe Previous keyframe (with preintegration).
     * @param reinit ORB `bRecInit`: when set, do not recover observations
     *        whose \(\chi^2\) is between the strict and loose thresholds.
     * @return Visual inlier count.
     */
    static int PoseInertialOptimizationLastKeyFrame(
        tracking::Frame* frame, const std::shared_ptr<KeyFrame>& last_keyframe,
        bool reinit = false);

    /**
     * @brief Visual-inertial pose opt vs last frame (ORB PoseInertialOptimizationLastFrame).
     * @param frame Current frame.
     * @param last_frame Previous frame.
     * @param reinit Same as `PoseInertialOptimizationLastKeyFrame`.
     * @return Visual inlier count.
     */
    static int PoseInertialOptimizationLastFrame(tracking::Frame* frame,
                                                 tracking::Frame* last_frame,
                                                 bool reinit = false);

    /**
     * @brief Refine scale and \(R_{wg}\) only (ORB third `InertialOptimization`).
     *
     * Poses, velocities and biases stay fixed. Inertial edges use a Huber
     * kernel with delta 1. Ten iterations.
     * @param map Map.
     * @param[in,out] Rwg Gravity direction.
     * @param[in,out] scale Scale.
     */
    static void ScaleRefinement(Map* map, Mat33* Rwg, double* scale);

    /**
     * @brief Optimize one body pose against point-to-plane factors.
     *
     * Optional absolute prior is a loose constraint (visual pose in LIVO).
     * Two rounds: solve, drop residuals larger than 0.2 m, solve again.
     * @param[in,out] T_wb Body pose in the world frame.
     * @param factors Plane correspondences. Fewer than 5 returns 0.
     * @param pose_prior Optional measured \(T_{wb}\). Null skips the prior.
     * @param prior_sqrt_info Square-root information of the prior.
     * @param max_iterations Ceres iteration cap per round.
     * @return Inlier count after the second round.
     */
    static int OptimizeLidarPose(SE3* T_wb,
                                 const std::vector<LidarPlaneFactor>& factors,
                                 const SE3* pose_prior, double prior_sqrt_info,
                                 int max_iterations);
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_OPTIMIZER_HPP_
