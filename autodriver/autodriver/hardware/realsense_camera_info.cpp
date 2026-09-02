/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief CameraInfo builder from RealSense intrinsics.
 */

#include "autodriver/hardware/realsense_camera_info.hpp"

namespace autodriver {
namespace hardware {
namespace realsense {

/** @brief Builds a CameraInfo message from RealSense intrinsics fields. */
automsgs::msgs::sensor_msgs::CameraInfo MakeCameraInfo(
    const std::uint32_t width, const std::uint32_t height,
    const std::string& frame_id, const double fx, const double fy,
    const double ppx, const double ppy, const float* coeffs,
    const int coeff_count) {
    automsgs::msgs::sensor_msgs::CameraInfo info;
    info.mutable_header()->set_frame_id(frame_id);
    info.set_width(width);
    info.set_height(height);
    info.set_distortion_model("plumb_bob");
    for (int i = 0; i < coeff_count; ++i) {
        info.add_d(coeffs[i]);
    }
    info.add_k(fx);
    info.add_k(0.0);
    info.add_k(ppx);
    info.add_k(0.0);
    info.add_k(fy);
    info.add_k(ppy);
    info.add_k(0.0);
    info.add_k(0.0);
    info.add_k(1.0);
    info.add_r(1.0);
    info.add_r(0.0);
    info.add_r(0.0);
    info.add_r(0.0);
    info.add_r(1.0);
    info.add_r(0.0);
    info.add_r(0.0);
    info.add_r(0.0);
    info.add_r(1.0);
    info.add_p(fx);
    info.add_p(0.0);
    info.add_p(ppx);
    info.add_p(0.0);
    info.add_p(0.0);
    info.add_p(fy);
    info.add_p(ppy);
    info.add_p(0.0);
    info.add_p(0.0);
    info.add_p(1.0);
    info.add_p(0.0);
    info.set_binning_x(1);
    info.set_binning_y(1);
    return info;
}

}  // namespace realsense
}  // namespace hardware
}  // namespace autodriver
