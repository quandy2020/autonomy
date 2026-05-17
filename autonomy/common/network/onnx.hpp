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

#ifndef AUTONOMY_COMMON_NETWORK_ONNX_HPP_
#define AUTONOMY_COMMON_NETWORK_ONNX_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/backend.hpp"
#include "autonomy/common/network/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file onnx.hpp
 * @brief ONNX Runtime backend (factory id "onnx")
 *
 * Requires BUILD_ONNXRUNTIME and AUTONOMY_HAS_ONNXRUNTIME. CPU EP, float32 I/O.
 */

/**
 * @brief @ref Backend implementation for .onnx models via ONNX Runtime
 */
class OnnxBackend : public Backend {
public:
    /**
     * @brief Constructs an unloaded backend (no active ORT session).
     */
    OnnxBackend();

    /**
     * @brief Destroys the backend and releases ORT session resources.
     */
    ~OnnxBackend() override;

    /**
     * @brief Move-constructs from another backend, transferring the session.
     */
    OnnxBackend(OnnxBackend&&) noexcept;

    /**
     * @brief Move-assigns from another backend, transferring the session.
     */
    OnnxBackend& operator=(OnnxBackend&&) noexcept;

    OnnxBackend(const OnnxBackend&) = delete;
    OnnxBackend& operator=(const OnnxBackend&) = delete;

    /**
     * @brief Loads an ONNX model from `model_path`.
     *
     * Equivalent to calling @ref LoadFromOptions with only `model_path` set.
     *
     * @param model_path Path to a `.onnx` file.
     * @return True if the session was created; false otherwise.
     */
    bool LoadFromFile(const std::string& model_path) override;

    /**
     * @brief Loads a model and applies @ref OnnxRuntimeOptions from `opt`.
     *
     * @param opt Must include a non-empty `model_path`; reads `opt.onnx` for ORT
     *            session settings.
     * @return True on success; false on failure (see @ref GetLastError).
     */
    bool LoadFromOptions(const InferenceOptions& opt) override;

    /**
     * @brief Returns whether an ORT session is active.
     *
     * @return True when a model is loaded and the session is valid.
     */
    bool IsLoaded() const override;

    /**
     * @brief Returns cached input tensor metadata from the loaded model.
     *
     * @return One @ref ModelTensorInfo per input; empty if not loaded.
     */
    std::vector<ModelTensorInfo> GetInputInfos() const override;

    /**
     * @brief Returns cached output tensor metadata from the loaded model.
     *
     * @return One @ref ModelTensorInfo per output; empty if not loaded.
     */
    std::vector<ModelTensorInfo> GetOutputInfos() const override;

    /**
     * @brief Runs ORT inference for all model outputs.
     *
     * Input names must match @ref GetInputInfos. Non-float32 model I/O types
     * are rejected until extended. Output tensors are materialized as float32
     * host buffers in `outputs`.
     *
     * @param inputs Named contiguous float32 buffers (row-major).
     * @param outputs Cleared and filled on success; must not be null.
     * @return True on success; false on validation or ORT errors.
     */
    bool Run(const std::unordered_map<std::string, std::vector<float>>& inputs,
             std::unordered_map<std::string, std::vector<float>>* outputs) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief Type alias for @ref OnnxBackend (legacy examples and demos).
 */
using OnnxEngine = OnnxBackend;

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_ONNX_HPP_
