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

#ifndef AUTONOMY_COMMON_NETWORK_PIPELINE_RUN_HPP_
#define AUTONOMY_COMMON_NETWORK_PIPELINE_RUN_HPP_

/**
 * @file run.hpp
 * @brief End-to-end pipeline: preprocess + @ref Engine::Run
 *
 * Wraps @ref Preprocess and forward inference in one call and returns
 * @ref TransformMeta for mapping network coordinates back to the source image
 * (@ref Decode, @ref ToMat, etc.).
 */

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/common/network/detail/preprocess/types.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>

namespace autonomy {
namespace common {
namespace network {

class Engine;

/**
 * @brief Combined output of @ref RunPipeline
 *
 * On success, `outputs` keys match @ref Engine::GetOutputInfos names.
 * `meta` / `meta_by_input` are meaningful only for image inputs.
 */
struct RunResult {
    TensorMap outputs;  //!< Model outputs (types as declared; often float32)
    TransformMeta
        meta;  //!< Geometry for the first image input (backward compatible)
    //!< Per image-input geometry when the model has multiple image tensors
    std::unordered_map<std::string, TransformMeta> meta_by_input;
};

/**
 * @brief Preprocess @p sample and run inference
 *
 * Steps:
 * 1. Verify @p engine has a loaded model
 * 2. @ref Preprocess(sample, GetInputInfos(), …) builds input @ref TensorMap
 * 3. @ref Engine::Run(inputs, &result->outputs)
 *
 * @param engine Loaded engine; must not be nullptr
 * @param sample Image, vector, or pre-filled tensors
 * @param options Preprocess policy (resize / normalize / layout)
 * @param[out] result Outputs and geometry metadata
 * @param[out] error Human-readable message on failure
 * @return false if any step fails
 */
bool RunPipeline(Engine* engine, const Sample& sample,
                 const PreprocessOptions& options, RunResult* result,
                 std::string* error = nullptr);

/**
 * @brief Preprocess a single BGR image and run inference
 *
 * Builds `Sample{ .image_bgr = image }` and calls the Sample overload.
 *
 * @param engine Loaded engine
 * @param image Non-empty BGR 8UC3 image
 * @param options Preprocess configuration
 * @param[out] result Outputs
 * @param[out] error Error message
 * @return true on success
 */
bool RunPipeline(Engine* engine, const cv::Mat& image,
                 const PreprocessOptions& options, RunResult* result,
                 std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PIPELINE_RUN_HPP_
