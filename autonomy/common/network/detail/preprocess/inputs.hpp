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
#include "autonomy/common/network/tensor.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file inputs.hpp
 * @brief Bind vector and pre-built named tensors to model inputs
 */

bool SetVector(const std::vector<float>& features, const ModelTensorInfo& info,
               TensorMap* out, std::string* error = nullptr);

bool SetNamed(const TensorMap& named, const std::vector<ModelTensorInfo>& infos,
              TensorMap* out, std::string* error = nullptr);

bool CheckSize(const ModelTensorInfo& info, size_t count, std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_INPUTS_HPP_
