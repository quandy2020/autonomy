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

#ifndef AUTONOMY_COMMON_NETWORK_TENSORRT_TENSORRT_HPP_
#define AUTONOMY_COMMON_NETWORK_TENSORRT_TENSORRT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/network/backend/backend.hpp"
#include "autonomy/common/network/common/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file tensorrt.hpp
 * @brief TensorRT backend (factory id `"tensorrt"`)
 *
 * Requires TensorRT/CUDA at build time. Builds from ONNX or loads a serialized
 * engine. Options: @ref InferenceOptions::tensorrt.
 */

/**
 * @brief @ref Backend that runs a TensorRT engine on CUDA
 *
 * Host buffers use @ref Tensor; Run performs H2D / inference / D2H on device.
 */
class TensorRtBackend : public Backend
{
public:
    TensorRtBackend();
    ~TensorRtBackend() override;

    TensorRtBackend(TensorRtBackend&&) noexcept;
    TensorRtBackend& operator=(TensorRtBackend&&) noexcept;

    TensorRtBackend(const TensorRtBackend&) = delete;
    TensorRtBackend& operator=(const TensorRtBackend&) = delete;

    bool LoadFromFile(const std::string& model_path) override;
    bool LoadFromOptions(const InferenceOptions& opt) override;
    bool IsLoaded() const override;
    std::vector<ModelTensorInfo> GetInputInfos() const override;
    std::vector<ModelTensorInfo> GetOutputInfos() const override;

    /**
     * @brief Runs TensorRT inference with typed host buffers on CUDA
     */
    bool Run(const TensorMap& inputs, TensorMap* outputs) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_TENSORRT_TENSORRT_HPP_
