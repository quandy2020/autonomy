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

#pragma once

#include <cstdint>
#include <string>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autodriver {
namespace hardware {
namespace realsense {

automsgs::msgs::sensor_msgs::CameraInfo MakeCameraInfo(
    std::uint32_t width, std::uint32_t height, const std::string& frame_id,
    double fx, double fy, double ppx, double ppy, const float* coeffs,
    int coeff_count);

}  // namespace realsense
}  // namespace hardware
}  // namespace autodriver
