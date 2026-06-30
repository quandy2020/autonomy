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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_BASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_BASE_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <vector>

namespace autonomy::localization::atlas {

namespace camera {
class base;
} // namespace camera

namespace data {
class frame;
} // namespace data

namespace initialize {

class base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    base() = delete;

    //! Constructor
    base(const data::frame& ref_frm,
         const unsigned int num_ransac_iters,
         const unsigned int min_num_valid_pts,
         const unsigned int min_num_triangulated,
         const float parallax_deg_thr,
         const float reproj_err_thr);

    //! Destructor
    virtual ~base() = default;

    //! Initialize with the current frame
    virtual bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) = 0;

    //! Get the rotation from the reference to the current
    Mat33_t get_rotation_ref_to_cur() const;

    //! Get the translation from the reference to the current
    Vec3_t get_translation_ref_to_cur() const;

    //! Get the triangulated 3D points with the origin of the reference frame
    eigen_alloc_vector<Vec3_t> get_triangulated_pts() const;

    //! Get the valid/invalid flags of triangulated 3D points as keypoint indices in the reference frame
    std::vector<bool> get_triangulated_flags() const;

protected:
    //! Find the most plausible pose and set them to the member variables (outputs)
    bool find_most_plausible_pose(const eigen_alloc_vector<Mat33_t>& init_rots, const eigen_alloc_vector<Vec3_t>& init_transes,
                                  const std::vector<bool>& is_inlier_match, const bool depth_is_positive);

    //! Generate 3D points from matches with valid and sufficient parallax
    unsigned int triangulate(const Mat33_t& rot_ref_to_cur, const Vec3_t& trans_ref_to_cur,
                             const std::vector<bool>& is_inlier_match, const bool depth_is_positive,
                             eigen_alloc_vector<Vec3_t>& triangulated_pts,
                             std::vector<bool>& is_triangulated,
                             unsigned int& num_triangulated_pts,
                             float& parallax_deg);

    //-----------------------------------------
    // reference frame information

    //! camera model of reference frame
    camera::base* const ref_camera_;
    //! undistorted keypoints of reference frame
    const std::vector<cv::KeyPoint> ref_undist_keypts_;
    //! bearing vectors of reference frame
    const eigen_alloc_vector<Vec3_t> ref_bearings_;

    //-----------------------------------------
    // current frame information

    //! camera matrix of current frame
    camera::base* cur_camera_;
    //! undistorted keypoints of current frame
    std::vector<cv::KeyPoint> cur_undist_keypts_;
    //! bearing vectors of current frame
    eigen_alloc_vector<Vec3_t> cur_bearings_;

    //-----------------------------------------
    // matching information

    //! matching between reference and current frames
    std::vector<std::pair<int, int>> ref_cur_matches_;

    //-----------------------------------------
    // parameters

    //! max number of iterations of RANSAC
    const unsigned int num_ransac_iters_;
    //! min number of triangulated pts
    const unsigned int min_num_triangulated_;
    //! min number of valid pts
    const unsigned int min_num_valid_pts_;
    //! min parallax
    const float parallax_deg_thr_;
    //! reprojection error threshold
    const float reproj_err_thr_;

    //-----------------------------------------
    // output variables

    //! initial rotation from reference to current
    Mat33_t rot_ref_to_cur_ = Mat33_t::Identity();
    //! initial translation from reference to current
    Vec3_t trans_ref_to_cur_ = Vec3_t::Zero();
    //! triangulated pts, with respect to indices of reference frame
    eigen_alloc_vector<Vec3_t> triangulated_pts_;
    //! each indices of reference frame is successfully triangulated or not
    std::vector<bool> is_triangulated_;
};

} // namespace initialize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_BASE_HPP_
