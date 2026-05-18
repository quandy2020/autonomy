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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_MAP_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_MAP_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file map.hpp
 * @brief Dense float tensor to OpenCV map and pseudo-color visualization
 */

/**
 * @brief Convert a dense float map tensor to CV_32FC1 at source image
 * resolution
 *
 * Uses trailing HxW from @p info when available (supports leading batch/channel
 * dims). Resizes to @p meta.source_height x source_width with linear
 * interpolation.
 *
 * @param output Row-major float buffer from the network
 * @param info Output tensor metadata
 * @param meta Preprocess geometry from @ref RunPipeline or @ref Preprocess
 * @param mat On success, single-channel float map
 * @param error Optional failure message
 * @return True on success
 */
bool ToMat(const std::vector<float>& output, const ModelTensorInfo& info,
           const TransformMeta& meta, cv::Mat* mat,
           std::string* error = nullptr);

/**
 * @brief Apply OpenCV colormap to a single-channel float image for display
 *
 * Min-max normalizes @p float_map to 8-bit, then applies a colormap.
 *
 * @param float_map Input CV_32FC1 map
 * @param bgr On success, BGR visualization image
 * @param colormap OpenCV colormap id; -1 uses COLORMAP_INFERNO
 * @return True on success
 */
bool Colorize(const cv::Mat& float_map, cv::Mat* bgr, int colormap = -1);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_MAP_HPP_
