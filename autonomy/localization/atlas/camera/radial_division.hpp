/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_CAMERA_RADIAL_DIVISION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_CAMERA_RADIAL_DIVISION_HPP_

#include "autonomy/localization/atlas/camera/base.hpp"

#include <opencv2/core/mat.hpp>

namespace autonomy::localization::atlas {
namespace camera {

// This class implements the camera model presented in:
//
//   "Simultaneous linear estimation of multiple view geometry and lens
//   distortion" by Andrew Fitzgibbon, CVPR 2001.
//
// The model is easy to implement and fast to evaluate.
// It is well suited for wide angle lenses like used in action cameras
// implemented by Steffen Urban, March 2020 (urbste@googlemail.com)
class radial_division final : public base {
public:
    radial_division(const std::string& name, const setup_type_t& setup_type, const color_order_t& color_order,
                    const unsigned int cols, const unsigned int rows, const double fps,
                    const double fx, const double fy, const double cx, const double cy,
                    const double distortion, const double focal_x_baseline = 0.0, const double depth_thr = 0.0);

    radial_division(const YAML::Node& yaml_node);

    ~radial_division() override;

    void show_parameters() const override final;

    image_bounds compute_image_bounds() const override final;

    cv::Point2f undistort_point(const cv::Point2f& dist_pt) const override final;

    Vec3_t convert_point_to_bearing(const cv::Point2f& undist_pt) const override final;

    cv::Point2f convert_bearing_to_point(const Vec3_t& bearing) const override final;

    bool reproject_to_image(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec2_t& reproj, float& x_right) const override final;

    bool reproject_to_bearing(const Mat33_t& rot_cw, const Vec3_t& trans_cw, const Vec3_t& pos_w, Vec3_t& reproj) const override final;

    nlohmann::json to_json() const override final;

    //-------------------------
    // Parameters specific to this model

    //! pinhole params
    const double fx_;
    const double fy_;
    const double cx_;
    const double cy_;
    const double fx_inv_;
    const double fy_inv_;

    //! distortion params
    const double distortion_;

    //! camera matrix in OpenCV format
    cv::Mat cv_cam_matrix_;
    //! camera matrix in Eigen format
    Mat33_t eigen_cam_matrix_;
};

} // namespace camera
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_CAMERA_RADIAL_DIVISION_HPP_
