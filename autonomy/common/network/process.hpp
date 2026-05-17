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

#ifndef AUTONOMY_COMMON_NETWORK_PROCESS_HPP_
#define AUTONOMY_COMMON_NETWORK_PROCESS_HPP_

#include "autonomy/common/network/detail/process.hpp"
#include "autonomy/common/network/tensor.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace autonomy {
namespace common {
namespace network {

class Engine;

/**
 * @file process.hpp
 * @brief Public preprocess, postprocess, and image inference pipeline API
 */

/**
 * @brief Result of @ref RunPipeline (model outputs and preprocess geometry)
 */
struct RunResult {
    TensorMap outputs;   //!< @brief Output name to row-major float32 tensor data
    TransformMeta meta;  //!< @brief Preprocess geometry for inverse mapping in postprocess
};

/**
 * @brief Preprocess one BGR image and run @ref Engine::Run
 *
 * Builds input tensors from @p image using @p options, executes inference, and
 * stores outputs in @p result. On success, @p result->meta records letterbox /
 * resize parameters for @ref Decode or @ref ToMat.
 *
 * @param engine Loaded inference engine; must not be null
 * @param image Input image in BGR, 8-bit, any size
 * @param options Resize, normalization, and layout policy
 * @param result On success, receives outputs and transform metadata; must not be null
 * @param error Optional human-readable message on failure
 * @return True on success; false if preprocess or inference fails
 */
bool RunPipeline(Engine* engine, const cv::Mat& image, const PreprocessOptions& options,
                 RunResult* result, std::string* error = nullptr);

/**
 * @brief Preprocess a @ref Sample and run @ref Engine::Run
 *
 * Fills model inputs from optional image, vector features, or pre-built
 * @ref TensorMap entries in @p sample, then runs inference. See @ref Preprocess
 * for per-field binding rules.
 *
 * @param engine Loaded inference engine; must not be null
 * @param sample Raw observation (image and/or named tensors)
 * @param options Image preprocess options when @p sample contains an image
 * @param result On success, receives outputs and transform metadata; must not be null
 * @param error Optional human-readable message on failure
 * @return True on success; false on failure
 */
bool RunPipeline(Engine* engine, const Sample& sample, const PreprocessOptions& options,
                 RunResult* result, std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PROCESS_HPP_
