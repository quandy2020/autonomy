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

#ifndef AUTONOMY_COMMON_NETWORK_ONNX_ONNX_HPP_
#define AUTONOMY_COMMON_NETWORK_ONNX_ONNX_HPP_

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/network/backend/backend.hpp"
#include "autonomy/common/network/common/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file onnx.hpp
 * @brief ONNX Runtime backend (factory id `"onnx"`)
 *
 * Loads `.onnx` models. Session options via @ref InferenceOptions::onnx.
 * Registration: `NetworkBackendTraits<OnnxBackend>`.
 */

/**
 * @brief @ref Backend using the ONNX Runtime C++ API
 *
 * - Load: creates `Ort::Session`, fills @ref ModelTensorInfo for I/O
 * - Run: CPU @ref Tensor ↔ `Ort::Value` (see `backend/onnx/io.hpp`)
 * - Supports float32 / float16 / int8 / int32 / int64 / uint8 I/O
 */
class OnnxBackend : public Backend
{
public:
    /** @brief Constructs backend without a loaded session */
    OnnxBackend();
    /** @brief Releases ORT session and related resources */
    ~OnnxBackend() override;

    OnnxBackend(OnnxBackend&&) noexcept;
    OnnxBackend& operator=(OnnxBackend&&) noexcept;

    OnnxBackend(const OnnxBackend&) = delete;
    OnnxBackend& operator=(const OnnxBackend&) = delete;

    bool LoadFromFile(const std::string& model_path) override;
    bool LoadFromOptions(const InferenceOptions& opt) override;
    bool IsLoaded() const override;
    std::vector<ModelTensorInfo> GetInputInfos() const override;
    std::vector<ModelTensorInfo> GetOutputInfos() const override;

    /**
     * @brief Runs ORT inference; I/O types must match @ref GetInputInfos
     */
    bool Run(const TensorMap& inputs, TensorMap* outputs) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_ONNX_ONNX_HPP_
