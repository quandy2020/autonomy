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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_OBSERVATION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_OBSERVATION_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace autonomy::localization::atlas {
namespace data {

struct frame_observation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame_observation() = default;
    frame_observation(const cv::Mat& descriptors,
                      const std::vector<cv::KeyPoint>& undist_keypts, const eigen_alloc_vector<Vec3_t>& bearings,
                      const std::vector<float>& stereo_x_right, const std::vector<float>& depths)
        : descriptors_(descriptors), undist_keypts_(undist_keypts), bearings_(bearings),
          stereo_x_right_(stereo_x_right), depths_(depths) {}

    //! descriptors
    cv::Mat descriptors_;
    //! undistorted keypoints of monocular or stereo left image
    std::vector<cv::KeyPoint> undist_keypts_;
    //! bearing vectors
    eigen_alloc_vector<Vec3_t> bearings_;
    //! disparities
    std::vector<float> stereo_x_right_;
    //! depths
    std::vector<float> depths_;
    //! keypoint indices in each of the cells
    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells_;
    //! number of columns of grid to accelerate reprojection matching
    unsigned int num_grid_cols_;
    //! number of rows of grid to accelerate reprojection matching
    unsigned int num_grid_rows_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_OBSERVATION_HPP_
