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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MATCH_STEREO_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MATCH_STEREO_HPP_

#include "autonomy/localization/atlas/match/base.hpp"

namespace autonomy::localization::atlas {

namespace data {
class frame;
} // namespace data

namespace match {

class stereo {
public:
    stereo() = delete;

    stereo(const std::vector<cv::Mat>& left_image_pyramid, const std::vector<cv::Mat>& right_image_pyramid,
           const std::vector<cv::KeyPoint>& keypts_left, const std::vector<cv::KeyPoint>& keypts_right,
           const cv::Mat& descs_left, const cv::Mat& descs_right,
           const std::vector<float>& scale_factors, const std::vector<float>& inv_scale_factors,
           const float focal_x_baseline, const float true_baseline);

    virtual ~stereo() = default;

    /**
     * Compute stereo matching in subpixel order
     */
    void compute(std::vector<float>& stereo_x_right, std::vector<float>& depths) const;

private:
    /**
     * Get the keypoints in the right image which are aligned according to y coordinates of the keypoints
     * @param margin
     * @return
     */
    std::vector<std::vector<unsigned int>> get_right_keypoint_indices_in_each_row(const float margin) const;

    /**
     * Find the closest right keypoint for each left keypoint in stereo
     * @param idx_left
     * @param scale_level_left
     * @param candidate_indices_right
     * @param min_x_right
     * @param max_x_right
     * @param best_idx_right
     * @param best_hamm_dist
     */
    void find_closest_keypoints_in_stereo(const unsigned int idx_left, const int scale_level_left,
                                          const std::vector<unsigned int>& candidate_indices_right,
                                          const float min_x_right, const float max_x_right,
                                          unsigned int& best_idx_right, unsigned int& best_hamm_dist) const;

    /**
     * Compute subpixel disparity using patch correlation and parabola fitting
     * @param keypt_left
     * @param keypt_right
     * @param best_x_right
     * @param best_disp
     * @param best_correlation
     * @return
     */
    bool compute_subpixel_disparity(const cv::KeyPoint& keypt_left, const cv::KeyPoint& keypt_right,
                                    float& best_x_right, float& best_disp, float& best_correlation) const;

    //! reference to left image pyramid
    const std::vector<cv::Mat>& left_image_pyramid_;
    //! reference to right image pyramid
    const std::vector<cv::Mat>& right_image_pyramid_;

    //! number of keypoints
    const unsigned int num_keypts_;
    //! reference to keypoints in left image
    const std::vector<cv::KeyPoint>& keypts_left_;
    //! reference to keypoints in right image
    const std::vector<cv::KeyPoint>& keypts_right_;

    //! reference to left descriptor
    const cv::Mat& descs_left_;
    //! reference to right descriptor
    const cv::Mat& descs_right_;

    //! reference to scale factors
    const std::vector<float>& scale_factors_;
    //! reference to inverse of scale factors
    const std::vector<float>& inv_scale_factors_;

    //! focal_x x baseline
    const float focal_x_baseline_;
    //! true baseline
    const float true_baseline_;

    //! minimum disparity
    const float min_disp_;
    //! maximum disparity
    const float max_disp_;

    //! maximum hamming distance
    static constexpr unsigned int hamm_dist_thr_ = (match::HAMMING_DIST_THR_HIGH + match::HAMMING_DIST_THR_LOW) / 2;
};

} // namespace match
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MATCH_STEREO_HPP_
