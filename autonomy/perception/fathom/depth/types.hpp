/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_PERCEPTION_FATHOM_DEPTH_TYPES_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_DEPTH_TYPES_HPP_

#include <opencv2/core.hpp>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @file types.hpp
 * @brief Public camera and aligned RGB-D input types for Fathom.
 */

/** Camera pinhole parameters in pixels at the input image resolution. */
struct CameraIntrinsics {
    float fx = 0.0F;
    float fy = 0.0F;
    float cx = 0.0F;
    float cy = 0.0F;
};

/**
 * Aligned sensor frame consumed by the Fathom depth refiner.
 *
 * `bgr` is CV_8UC3 and `raw_depth` is CV_16UC1 in sensor depth units. The
 * caller supplies `depth_scale` to convert raw-depth units to meters.
 */
struct DepthInput {
    cv::Mat bgr;
    cv::Mat raw_depth;
    CameraIntrinsics intrinsics;
};

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_DEPTH_TYPES_HPP_
