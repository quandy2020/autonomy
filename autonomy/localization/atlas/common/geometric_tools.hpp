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
 * GeometricTools adapted from ORB-SLAM3 GeometricTools.
 */

/**
 * @file geometric_tools.hpp
 * @brief Geometry helpers: fundamental matrix, DLT triangulation, epipolar
 *        constraints (ORB-SLAM3 GeometricTools).
 *
 * Used by frontend matching / map initialization; depends on `map::KeyFrame`
 * and `sensor::GeometricCamera`. Stateless static utilities; thread safety
 * depends on read-only use of the passed objects.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_COMMON_GEOMETRIC_TOOLS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_COMMON_GEOMETRIC_TOOLS_HPP_

#include <memory>

#include <opencv2/core/types.hpp>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::GeometricTools
 * @brief Two-view geometry: F matrix, triangulation, epipolar (ORB-SLAM3 aligned).
 *
 * Typical use:
 * 1. `ComputeF12` for the fundamental matrix;
 * 2. `Triangulate` / `TriangulatePixels` for 3D points;
 * 3. Fisheye: `TriangulateMatches` / `EpipolarConstrain`.
 */
class GeometricTools {
public:
    /**
     * @brief Compute fundamental matrix between two keyframes
     *        @f$ F_{12}=K_1^{-T}[t]_{\times} R K_2^{-1} @f$.
     * @param keyframe1 View 1; returns Identity if null.
     * @param keyframe2 View 2; returns Identity if null.
     * @return 3x3 fundamental matrix; `Identity` on invalid input.
     */
    static Mat33 ComputeF12(
        const std::shared_ptr<KeyFrame>& keyframe1,
        const std::shared_ptr<KeyFrame>& keyframe2);

    /**
     * @brief DLT triangulation from normalized rays and 3x4 projection poses.
     * @param x_c1 Camera-1 normalized bearing @f$ (x,y,1) @f$ or unit ray.
     * @param x_c2 Camera-2 normalized bearing.
     * @param Tc1w Camera 1: world → camera 3x4 matrix.
     * @param Tc2w Camera 2: world → camera 3x4 matrix.
     * @param[out] x3d Triangulated world point; must not be nullptr.
     * @return true on success; false if `x3d==nullptr` or numeric failure.
     */
    static bool Triangulate(const Vec3& x_c1, const Vec3& x_c2,
                            const Eigen::Matrix<double, 3, 4>& Tc1w,
                            const Eigen::Matrix<double, 3, 4>& Tc2w, Vec3* x3d);

    /**
     * @brief Pixel keypoints → world point (via KF poses and intrinsics K).
     * @param uv1 View-1 pixel.
     * @param uv2 View-2 pixel.
     * @param keyframe1 Provides pose and fx,fy,cx,cy.
     * @param keyframe2 Same for view 2.
     * @param[out] x3d_world World point; must not be nullptr.
     * @return true on success; false if null pointer or triangulation fails.
     */
    static bool TriangulatePixels(const cv::Point2f& uv1,
                                  const cv::Point2f& uv2,
                                  const std::shared_ptr<KeyFrame>& keyframe1,
                                  const std::shared_ptr<KeyFrame>& keyframe2,
                                  Vec3* x3d_world);

    /**
     * @brief Explicit camera / pose triangulation (fisheye stereo ll|lr|rl|rr, etc.).
     * @param uv1 View-1 pixel.
     * @param uv2 View-2 pixel.
     * @param cam1 Geometric camera 1; may be nullptr (falls back to fx1..cy1).
     * @param cam2 Geometric camera 2; may be nullptr.
     * @param Tcw1 world → camera1.
     * @param Tcw2 world → camera2.
     * @param fx1 Camera-1 fx (chosen vs cam by implementation).
     * @param fy1 Camera-1 fy.
     * @param cx1 Camera-1 cx.
     * @param cy1 Camera-1 cy.
     * @param fx2 Camera-2 fx.
     * @param fy2 Camera-2 fy.
     * @param cx2 Camera-2 cx.
     * @param cy2 Camera-2 cy.
     * @param[out] x3d_world World point.
     * @return true on success; false on failure.
     */
    static bool TriangulatePixels(
        const cv::Point2f& uv1, const cv::Point2f& uv2,
        const sensor::GeometricCamera* cam1,
        const sensor::GeometricCamera* cam2, const SE3& Tcw1, const SE3& Tcw2,
        float fx1, float fy1, float cx1, float cy1, float fx2, float fy2,
        float cx2, float cy2, Vec3* x3d_world);

    /**
     * @brief Epipolar constraint check (ORB `GeometricCamera::epipolarConstrain`).
     * @param cam1 Camera-1 model.
     * @param cam2 Camera-2 model.
     * @param kp1 View-1 keypoint (octave enters uncertainty).
     * @param kp2 View-2 keypoint.
     * @param R12 Relative rotation: camera1 ← camera2.
     * @param t12 Relative translation: camera1 ← camera2.
     * @param sigma_level Scale-level uncertainty.
     * @param unc Reprojection / epipolar uncertainty threshold.
     * @return true if the constraint holds; false otherwise.
     */
    static bool EpipolarConstrain(const sensor::GeometricCamera* cam1,
                                  const sensor::GeometricCamera* cam2,
                                  const cv::KeyPoint& kp1,
                                  const cv::KeyPoint& kp2, const Mat33& R12,
                                  const Vec3& t12, float sigma_level,
                                  float unc);

    /**
     * @brief Fisheye match triangulation (ORB `KannalaBrandt8::TriangulateMatches`).
     *
     * Unproject rays + DLT + reprojection checks.
     * @param cam1 Camera-1 model.
     * @param cam2 Camera-2 model.
     * @param kp1 View-1 keypoint.
     * @param kp2 View-2 keypoint.
     * @param R12 Relative rotation.
     * @param t12 Relative translation.
     * @param sigma_level Scale-level uncertainty.
     * @param unc Reprojection threshold.
     * @param[out] p3d_cam1 3D point in camera-1 frame; may be nullptr (depth only).
     * @return Camera-1 depth (>0 on success); ≤0 on failure.
     */
    static float TriangulateMatches(
        const sensor::GeometricCamera& cam1,
        const sensor::GeometricCamera& cam2, const cv::KeyPoint& kp1,
        const cv::KeyPoint& kp2, const Mat33& R12, const Vec3& t12,
        float sigma_level, float unc, Vec3* p3d_cam1);
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_COMMON_GEOMETRIC_TOOLS_HPP_
