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

#ifndef AUTONOMY_PERCEPTION_FATHOM_PROCESSING_RGBD_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_PROCESSING_RGBD_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/fathom/depth/types.hpp"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @file rgbd.hpp
 * @brief RGB-D frame conversion to the fixed Fathom ONNX input contract.
 */

/**
 * Convert an aligned BGR8/depth16 frame to fixed-profile Fathom tensors.
 *
 * The output map contains float32 `image` with shape `[1, 3, height, width]`
 * in planar RGB order normalized to [0, 1], and float32 `raw_depth` with
 * shape `[1, height, width]` in meters. A raw-depth value of zero remains
 * zero. Camera intrinsics stay in the original image pixel frame because the
 * fixed ONNX graph accepts only `image` and `raw_depth`; projection happens
 * after the refiner restores outputs to that original resolution.
 *
 * @param input Aligned CV_8UC3 BGR and CV_16UC1 raw-depth images
 * @param width Fixed model input width, greater than zero
 * @param height Fixed model input height, greater than zero
 * @param depth_scale Sensor-depth-unit to meter scale, finite and positive
 * @param tensors Output tensors named `image` and `raw_depth`
 * @param error Optional failure message
 * @return True when both tensors are produced
 */
bool PrepareRgbd(const DepthInput& input, int width, int height,
                 float depth_scale, common::network::TensorMap* tensors,
                 std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_PROCESSING_RGBD_HPP_
