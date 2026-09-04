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

#ifndef AUTONOMY_PERCEPTION_FATHOM_PROJECTION_POINT_CLOUD_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_PROJECTION_POINT_CLOUD_HPP_

#include "autonomy/perception/fathom/depth/types.hpp"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @file point_cloud.hpp
 * @brief Metric depth projection into an organized camera-frame point cloud.
 */

/**
 * Project metric depth and its validity mask into a CV_32FC3 XYZ image.
 *
 * Each valid `(u, v)` position becomes
 * `((u - cx) * z / fx, (v - cy) * z / fy, z)`. Masked, non-finite, and
 * non-positive depth pixels are represented by NaN in all three coordinates.
 *
 * @param depth_m CV_32FC1 metric depth image in meters
 * @param mask CV_8UC1 validity mask; non-zero values are valid
 * @param intrinsics Original image pixel intrinsics with positive focal lengths
 * @param xyz Output organized CV_32FC3 camera-frame points
 * @param error Optional failure message
 * @return True when projection succeeds
 */
bool ProjectDepth(const cv::Mat& depth_m, const cv::Mat& mask,
                  const CameraIntrinsics& intrinsics, cv::Mat* xyz,
                  std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_PROJECTION_POINT_CLOUD_HPP_
