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

#ifndef AUTONOMY_COMMON_NETWORK_PREPROCESS_IMAGE_HPP_
#define AUTONOMY_COMMON_NETWORK_PREPROCESS_IMAGE_HPP_

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
 * @file image.hpp
 * @brief BGR image to model input tensor conversion
 */

/**
 * @brief Build one image input tensor from a BGR image
 *
 * Pipeline: @ref Resize → blob + @ref internal::ApplyNorm →
 * @ref internal::ToLayout → @ref internal::Expand.
 *
 * @param image_bgr Source image in BGR; must be non-empty
 * @param input Model input metadata from @ref Engine::GetInputInfos
 * @param options Preprocess configuration
 * @param tensor On success, row-major float32 buffer sized to the model input
 * @param meta Optional geometry for postprocess; may be null
 * @param error Optional failure message
 * @return True on success
 */
bool Preprocess(const cv::Mat& image_bgr, const ModelTensorInfo& input,
                const PreprocessOptions& options, std::vector<float>* tensor,
                TransformMeta* meta = nullptr, std::string* error = nullptr);

/**
 * @brief Build all model input tensors from a @ref Sample
 *
 * Binds image tensors, vector inputs, and pre-filled @ref TensorMap entries to
 * each @ref ModelTensorInfo in @p inputs. Fails if any required input is
 * missing.
 *
 * @param sample Observation data
 * @param inputs Model input descriptors
 * @param options Image preprocess options when sample contains an image
 * @param tensors On success, map of input name to float buffer
 * @param meta Optional geometry for the first image input (backward compatible)
 * @param meta_by_input Optional map of input name to geometry for each image
 * tensor
 * @param error Optional failure message
 * @return True on success
 */
bool Preprocess(
    const Sample& sample, const std::vector<ModelTensorInfo>& inputs,
    const PreprocessOptions& options, TensorMap* tensors,
    TransformMeta* meta = nullptr,
    std::unordered_map<std::string, TransformMeta>* meta_by_input = nullptr,
    std::string* error = nullptr);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_PREPROCESS_IMAGE_HPP_
