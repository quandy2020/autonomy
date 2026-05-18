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

#ifndef AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_SHAPE_HPP_
#define AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_SHAPE_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"
#include "autonomy/common/network/common/tensor.hpp"

#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {
namespace internal {

/**
 * @file shape.hpp
 * @brief Model input shape parsing and tensor batch expansion
 */

/**
 * @brief Parses spatial size and layout from a model image input descriptor
 *
 * Supports 4-D NCHW/NHWC and 5-D batched layouts. On failure to parse rank,
 * leaves @p shape at defaults derived from @p default_h and @p default_w.
 *
 * @param info Model input tensor metadata
 * @param default_h Fallback height when a dimension is dynamic or missing
 * @param default_w Fallback width when a dimension is dynamic or missing
 * @param shape Output parsed height, width, channels, and layout
 * @param error Optional failure reason; must not be null on hard errors
 * @return True when rank is 4 or 5 and parsing succeeded; false for unsupported rank
 */
bool ParseShape(const ModelTensorInfo& info, int default_h, int default_w,
                ImageInputShape* shape, std::string* error = nullptr);

/**
 * @brief Collapses a 5-D image input view to 4-D NCHW for single-batch preprocessing
 *
 * @param info Original model input descriptor
 * @param view On success, receives a 4-D view suitable for @ref Preprocess
 * @param error Optional failure reason
 * @return True on success; false when rank is unsupported
 */
bool Collapse(const ModelTensorInfo& info, ModelTensorInfo* view, std::string* error = nullptr);

/**
 * @brief Replicates a single-sample tensor to match a batched model input shape
 *
 * When the model expects a fixed batch dimension and the caller supplies one
 * sample, duplicates the buffer along the batch axis.
 *
 * @param info Target model input descriptor (defines expected element count)
 * @param tensor In/out buffer; resized and tiled on success
 * @param error Optional failure reason when sizes are incompatible
 * @return True on success or when no expansion is required
 */
bool Expand(const ModelTensorInfo& info, std::vector<float>* tensor, std::string* error = nullptr);

}  // namespace internal
}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_DETAIL_INTERNAL_SHAPE_HPP_
