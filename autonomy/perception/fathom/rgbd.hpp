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

/**
 * @file rgbd.hpp
 * @brief RGB-D conversion for the fixed Fathom model input contract.
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_RGBD_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_RGBD_HPP_

#include "autonomy/common/network/common/tensor.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @brief Converts aligned RGB and depth messages to fixed-profile tensors.
 *
 * The output map contains float32 `image` with shape `[1, 3, height, width]`
 * in planar RGB order normalized to [0, 1], and float32 `raw_depth` with
 * shape `[1, height, width]` in meters. `depth_scale` multiplies the numeric
 * samples in either supported depth encoding: use `0.001` for millimeter
 * `16UC1`, and `1.0` for Autosim's metric `32FC1`. Non-finite and non-positive
 * float depth samples are sanitized to zero. Camera intrinsics stay in the
 * original image pixel frame because the fixed graph accepts only `image` and
 * `raw_depth`; projection happens after output restoration.
 *
 * @param rgb Aligned `bgr8` or `rgb8` image message.
 * @param raw_depth Aligned `16UC1` or `32FC1` raw-depth image message.
 * @param width Fixed model input width, greater than zero.
 * @param height Fixed model input height, greater than zero.
 * @param depth_scale Sensor-depth-unit to meter scale, finite and positive.
 * @param tensors Output tensors named `image` and `raw_depth`; cleared first.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when both input tensors are produced.
 */
bool PrepareRgbd(const automsgs::msgs::sensor_msgs::Image& rgb,
                 const automsgs::msgs::sensor_msgs::Image& raw_depth, int width,
                 int height, float depth_scale,
                 common::network::TensorMap* tensors,
                 std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_RGBD_HPP_
