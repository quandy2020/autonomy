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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_INPUTS_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_INPUTS_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"
#include "autonomy/common/network/common/tensor.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file inputs.hpp
 * @brief Non-image input binding: vector features and pre-filled @ref TensorMap
 *
 * Called from @ref Preprocess(const Sample&, …) outside the image path.
 */

/**
 * @brief Writes a 1-D feature vector into a vector input slot
 *
 * @param features Feature values; length must match static shape (see @ref CheckSize)
 * @param info Target input descriptor
 * @param[out] out Accumulated input map
 * @param[out] error Element count mismatch, etc.
 */
bool SetVector(const std::vector<float>& features, const ModelTensorInfo& info,
               TensorMap* out, std::string* error = nullptr);

/**
 * @brief Binds pre-built tensors from @p named to model inputs by name
 *
 * @param named Caller tensors (any @ref ElementType)
 * @param infos All model input descriptors
 * @param[out] out Output map
 * @param[out] error Missing required input or size error
 */
bool SetNamed(const TensorMap& named, const std::vector<ModelTensorInfo>& infos,
              TensorMap* out, std::string* error = nullptr);

/**
 * @brief Verifies buffer element count against static shape
 *
 * @param info Model tensor metadata
 * @param count Actual element count
 * @param[out] error Mismatch message
 */
bool CheckSize(const ModelTensorInfo& info, size_t count, std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_INPUTS_HPP_
