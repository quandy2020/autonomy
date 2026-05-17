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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_INTERNAL_SHAPE_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_INTERNAL_SHAPE_HPP_

#include "autonomy/common/network/detail/preprocess/types.hpp"
#include "autonomy/common/network/tensor.hpp"

#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {
namespace preprocess_internal {

/**
 * @file shape.hpp
 * @brief Internal tensor shape parsing and batch expansion
 */

bool ParseShape(const ModelTensorInfo& info, int default_h, int default_w,
                ImageInputShape* shape, std::string* error = nullptr);

bool Collapse(const ModelTensorInfo& info, ModelTensorInfo* view, std::string* error = nullptr);

bool Expand(const ModelTensorInfo& info, std::vector<float>* tensor, std::string* error = nullptr);

}  // namespace preprocess_internal
}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_INTERNAL_SHAPE_HPP_
