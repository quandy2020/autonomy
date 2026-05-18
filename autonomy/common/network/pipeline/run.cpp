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

#include "autonomy/common/network/pipeline/run.hpp"

#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/common/network/detail/internal/error.hpp"
#include "autonomy/common/network/detail/preprocess/image.hpp"

namespace autonomy {
namespace common {
namespace network {

namespace {

using internal::SetErrorMessage;

bool RunPipelineImpl(Engine* engine, const Sample& sample,
                     const PreprocessOptions& options, RunResult* result,
                     std::string* error) {
    if (engine == nullptr) {
        SetErrorMessage(error, "engine is null.");
        return false;
    }
    if (result == nullptr) {
        SetErrorMessage(error, "result is null.");
        return false;
    }
    result->outputs.clear();
    result->meta = TransformMeta{};
    result->meta_by_input.clear();

    const auto& input_infos = engine->GetInputInfos();
    if (input_infos.empty()) {
        SetErrorMessage(error, "model has no inputs.");
        return false;
    }

    TensorMap inputs;
    if (!Preprocess(sample, input_infos, options, &inputs, &result->meta,
                    &result->meta_by_input, error)) {
        return false;
    }

    if (!engine->Run(inputs, &result->outputs)) {
        SetErrorMessage(error, engine->GetLastError());
        return false;
    }
    return true;
}

}  // namespace

bool RunPipeline(Engine* engine, const cv::Mat& image,
                 const PreprocessOptions& options, RunResult* result,
                 std::string* error) {
    if (image.empty()) {
        SetErrorMessage(error, "empty image.");
        return false;
    }
    Sample sample;
    sample.image_bgr = image;
    return RunPipelineImpl(engine, sample, options, result, error);
}

bool RunPipeline(Engine* engine, const Sample& sample,
                 const PreprocessOptions& options, RunResult* result,
                 std::string* error) {
    return RunPipelineImpl(engine, sample, options, result, error);
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
