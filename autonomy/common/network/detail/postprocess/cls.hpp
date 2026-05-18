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

#ifndef AUTONOMY_COMMON_NETWORK_POSTPROCESS_CLS_HPP_
#define AUTONOMY_COMMON_NETWORK_POSTPROCESS_CLS_HPP_

#include "autonomy/common/network/detail/postprocess/types.hpp"

#include <string>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file cls.hpp
 * @brief Classification logits top-k extraction
 */

/**
 * @brief Select the top-k class indices by score from a logits vector
 *
 * @param logits Per-class scores (length = number of classes)
 * @param top_k Number of results to return (clamped to logits size)
 * @param result On success, sorted by descending score
 * @param error Optional failure message
 * @return True on success
 */
bool TopK(const std::vector<float>& logits, int top_k,
          std::vector<ClassScore>* result, std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_POSTPROCESS_CLS_HPP_
