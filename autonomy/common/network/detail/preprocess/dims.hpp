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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_DIMS_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_DIMS_HPP_

#include "autonomy/common/network/common/tensor.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file dims.hpp
 * @brief Model input dimension parsing and input-type classification
 *
 * Distinguishes image inputs (4-D/5-D) from vector inputs (1-D/2-D) before preprocess,
 * and extracts spatial height/width from @ref ModelTensorInfo.
 */

/**
 * @brief Read one dimension with a default when missing or non-positive
 * @param dims Full dimension list
 * @param index Dimension index
 * @param fallback Value when @p index is out of range or dim <= 0
 */
int Dim(const std::vector<int64_t>& dims, size_t index, int fallback);

/**
 * @brief True when the tensor looks like a 4-D or 5-D image input
 */
bool IsImage(const ModelTensorInfo& info);

/**
 * @brief True when the tensor looks like a 1-D or 2-D vector input
 */
bool IsVector(const ModelTensorInfo& info);

/**
 * @brief Query spatial height and width from an image input tensor
 * @param info Model input descriptor
 * @param fallback Used when shape cannot be parsed
 * @param height Output height
 * @param width Output width
 * @return True when parsed from metadata; false if @p fallback was used
 */
bool GetSpatialSize(const ModelTensorInfo& info, int fallback, int* height, int* width);

namespace internal {

/** @brief True when a dimension value is a typical channel count (1 or 3) */
inline bool IsChannelDim(int64_t dim) { return dim == 1 || dim == 3; }

}  // namespace internal

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_DIMS_HPP_
