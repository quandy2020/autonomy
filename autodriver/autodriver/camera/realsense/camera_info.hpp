/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file camera_info.hpp
 * @brief Build sensor_msgs/CameraInfo from RealSense intrinsics.
 */

#pragma once

#include <cstdint>
#include <string>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autodriver {
namespace hardware {
namespace realsense {

/**
 * @brief Fill CameraInfo K/D from pinhole intrinsics (+ optional distortion).
 * @param[in] width Image width in pixels.
 * @param[in] height Image height in pixels.
 * @param[in] frame_id Header frame_id written into CameraInfo.
 * @param[in] fx Focal length x (pixels).
 * @param[in] fy Focal length y (pixels).
 * @param[in] ppx Principal point x (pixels).
 * @param[in] ppy Principal point y (pixels).
 * @param[in] coeffs Distortion coefficients; may be null when @p coeff_count == 0.
 * @param[in] coeff_count Number of entries in @p coeffs.
 * @return Filled CameraInfo message (K, D, size, frame_id).
 */
automsgs::msgs::sensor_msgs::CameraInfo MakeCameraInfo(
    std::uint32_t width, std::uint32_t height, const std::string& frame_id,
    double fx, double fy, double ppx, double ppy, const float* coeffs,
    int coeff_count);

}  // namespace realsense
}  // namespace hardware
}  // namespace autodriver
