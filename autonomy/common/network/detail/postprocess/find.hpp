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

#ifndef AUTONOMY_COMMON_NETWORK_DETAIL_POSTPROCESS_FIND_HPP_
#define AUTONOMY_COMMON_NETWORK_DETAIL_POSTPROCESS_FIND_HPP_

#include "autonomy/common/network/common/tensor.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace autonomy {
namespace common {
namespace network {

/**
 * @file find.hpp
 * @brief Select one float32 output from a multi-output @ref TensorMap (no
 * OpenCV)
 *
 * Selection order:
 * 1. Exact name match when @p keyword is non-empty
 * 2. First name containing @p keyword (case-insensitive) with a float32 view
 * 3. Largest spatial (H×W) float32 tensor among all outputs
 * 4. First float32-viewable output in the map
 */

/**
 * @brief Finds a single float32 output tensor
 *
 * @param outputs Result of @ref Engine::Run
 * @param infos @ref Engine::GetOutputInfos (spatial-size heuristic)
 * @param[out] name Selected output name
 * @param[out] data Non-owning pointer to internal float data
 * @param keyword Substring or exact name; empty skips name-based heuristics
 * @param[out] error Message when no output or not float32
 * @return true on success
 */
bool FindFloatOutput(const TensorMap& outputs,
                     const std::vector<ModelTensorInfo>& infos,
                     std::string* name, const float** data,
                     const std::string& keyword = "",
                     std::string* error = nullptr);

/**
 * @brief Legacy helper: selects a float32 output and exposes it as @ref
 * std::vector<float>
 *
 * Data is copied into internal storage; valid until the next @ref Find call on
 * the same thread. Prefer @ref FindFloatOutput for zero-copy views.
 */
bool Find(const TensorMap& outputs, const std::vector<ModelTensorInfo>& infos,
          std::string* name, const std::vector<float>** data,
          const std::string& keyword = "", std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_DETAIL_POSTPROCESS_FIND_HPP_
