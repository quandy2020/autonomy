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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_PERSPECTIVE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_PERSPECTIVE_HPP_

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/initialize/base.hpp"

namespace autonomy::localization::atlas {

namespace data {
class frame;
} // namespace data

namespace initialize {

class perspective final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    perspective() = delete;

    //! Constructor
    perspective(const data::frame& ref_frm,
                const unsigned int num_ransac_iters,
                const unsigned int min_num_triangulated,
                const unsigned int min_num_valid_pts,
                const float parallax_deg_thr,
                const float reproj_err_thr,
                bool use_fixed_seed = false);

    //! Destructor
    ~perspective() override;

    //! Initialize with the current frame
    bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;

private:
    //! Reconstruct the initial map with the H matrix
    //! (NOTE: the output variables will be set if succeeded)
    bool reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match);

    //! Reconstruct the initial map with the F matrix
    //! (NOTE: the output variables will be set if succeeded)
    bool reconstruct_with_F(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match);

    //! Get the camera matrix from the camera object
    static Mat33_t get_camera_matrix(camera::base* camera);

    //! camera matrix of the reference frame
    const Mat33_t ref_cam_matrix_;
    //! camera matrix of the current frame
    Mat33_t cur_cam_matrix_;

    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;
};

} // namespace initialize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_PERSPECTIVE_HPP_
