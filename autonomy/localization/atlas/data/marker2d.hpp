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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_MARKER2D_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_MARKER2D_HPP_

#include "autonomy/localization/atlas/type.hpp"
#include <opencv2/core/types.hpp>
#include <Eigen/Core>

namespace autonomy::localization::atlas {
namespace marker_model {
class base;
}

namespace data {

class marker2d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    marker2d(const std::vector<cv::Point2f>& undist_corners, const eigen_alloc_vector<Vec3_t>& bearings,
             const Mat33_t& rot_cm, const Vec3_t& trans_cm, unsigned int id, const std::shared_ptr<marker_model::base>& marker_model,
             const std::vector<cv::Point2f>& dist_corners);

    //! Compute corner positions on the world from camera pose and corner positions on the camera
    eigen_alloc_vector<Vec3_t> compute_corners_pos_w(const Mat44_t& cam_pose_wc, const eigen_alloc_vector<Vec3_t>& corners_pos) const;

    //! undistorted corner points
    std::vector<cv::Point2f> undist_corners_;

    //! bearing of corners
    eigen_alloc_vector<Vec3_t> bearings_;

    //! marker pose (camera -> marker)
    Mat33_t rot_cm_;
    Vec3_t trans_cm_;

    //! marker ID
    unsigned int id_;

    //! marker model
    std::shared_ptr<marker_model::base> marker_model_;

    std::vector<cv::Point2f> dist_corners_; // Keep these to draw the markers
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_MARKER2D_HPP_
