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
 * Monocular two-view init (ORB-SLAM3 TwoViewReconstruction API surface;
 * OpenCV essential-matrix path as first implementation).
 */

/**
 * @file two_view_reconstruction.hpp
 * @brief Monocular two-view init: recover relative pose T_21 and triangulate 3D points.
 *
 * API aligned with ORB-SLAM3 TwoViewReconstruction; current impl uses OpenCV
 * essential-matrix path.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_TWO_VIEW_RECONSTRUCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_TWO_VIEW_RECONSTRUCTION_HPP_

#include <vector>

#include <opencv2/core/types.hpp>

#include "Eigen/Core"

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace tracking {

/**
 * @class autonomy::localization::atlas::tracking::TwoViewReconstruction
 * @brief Two-view geometric reconstructor for monocular initialization.
 *
 * Inputs: keypoints of two frames and matches12; outputs SE3 of frame2 relative
 * to frame1 and 3D points in the world / camera-1 convention of the impl.
 *
 * @note Called by Tracker::MonocularInitialization on the Tracking thread.
 */
class TwoViewReconstruction {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct the reconstructor.
     * @param K Camera intrinsics matrix (3×3).
     * @param sigma Pixel noise scale (reprojection threshold related), default 1.
     * @param iterations RANSAC / iteration cap, default 200.
     */
    explicit TwoViewReconstruction(const Mat33& K, float sigma = 1.f,
                                   int iterations = 200);

    /**
     * @brief Recover relative pose from two-view matches and triangulate.
     * @param keys1 Keypoints of the first frame.
     * @param keys2 Keypoints of the second frame.
     * @param matches12 Same length as keys1: matches12[i]=j means keys1[i]↔keys2[j];
     *                  negative if unmatched.
     * @param[out] T21 Pose of frame2 relative to frame1 (impl convention).
     * @param[out] points3d Triangulated 3D points (camera-1 / world init convention).
     * @param[out] triangulated Success mask aligned with matches.
     * @return true if geometry validates and outputs are valid; else false.
     */
    bool Reconstruct(const std::vector<cv::KeyPoint>& keys1,
                     const std::vector<cv::KeyPoint>& keys2,
                     const std::vector<int>& matches12, SE3* T21,
                     std::vector<cv::Point3f>* points3d,
                     std::vector<bool>* triangulated) const;

private:
    Mat33 K_ = Mat33::Identity();  ///< Camera intrinsics
    float sigma_ = 1.f;            ///< Pixel noise scale
    int iterations_ = 200;         ///< RANSAC / iteration cap
};

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_TWO_VIEW_RECONSTRUCTION_HPP_
