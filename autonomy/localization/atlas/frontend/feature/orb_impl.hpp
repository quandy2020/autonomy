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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FEATURE_ORB_IMPL_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FEATURE_ORB_IMPL_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace autonomy::localization::atlas {
namespace feature {

class orb_impl {
public:
    orb_impl();
    float ic_angle(const cv::Mat& image, const cv::Point2f& point) const;
    void compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, uchar* desc) const;

    //! BRIEF orientation
    static constexpr unsigned int fast_patch_size_ = 31;
    //! half size of FAST patch
    static constexpr int fast_half_patch_size_ = fast_patch_size_ / 2;

private:
    //! Index limitation that used for calculating of keypoint orientation
    std::vector<int> u_max_;
};

} // namespace feature
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FEATURE_ORB_IMPL_HPP_
