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

#ifndef AUTONOMY_COMMON_NETWORK_BACKEND_ENGINE_HPP_
#define AUTONOMY_COMMON_NETWORK_BACKEND_ENGINE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/common/options.hpp"
#include "autonomy/common/network/common/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {

class Backend;

/**
 * @file engine.hpp
 * @brief Public entry point for neural network model inference
 */

/**
 * @brief Model inference engine (ONNX Runtime or TensorRT)
 *
 * Select backend via @ref InferenceOptions::backend_id (`"onnx"` or `"tensorrt"`).
 * Does not perform image preprocessing; use @ref RunPipeline (include network.hpp).
 *
 * @note Not thread-safe: do not call @ref Run concurrently on the same @ref Engine
 *       from multiple threads unless you provide external synchronization.
 */
class Engine {
public:
    /**
     * @brief Constructs an engine with no backend (not usable until created
     *        via @ref CreateEngine or assigned from a moved-from engine).
     */
    Engine();

    /**
     * @brief Destroys the engine and releases the owned backend.
     */
    ~Engine();

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    /**
     * @brief Move-constructs an engine, transferring backend ownership.
     */
    Engine(Engine&&) noexcept;

    /**
     * @brief Move-assigns an engine, transferring backend ownership.
     */
    Engine& operator=(Engine&&) noexcept;

    /**
     * @brief Creates an engine and loads a model
     *
     * @param opt backend_id, model_path, and backend-specific options
     * @param error_message If non-null, filled when this returns nullptr
     * @return A loaded engine on success, or nullptr on failure
     */
    static std::unique_ptr<Engine> CreateEngine(
        const InferenceOptions& opt, std::string* error_message = nullptr);

    /**
     * @brief Creates an engine from a model path
     *
     * @param model_path `.onnx` (ORT / TRT build) or `.engine` / `.plan` (TRT)
     * @param backend_id `"onnx"` (default) or `"tensorrt"`
     * @param error_message If non-null, filled when creation fails
     * @return A loaded engine on success, or nullptr on failure
     */
    static std::unique_ptr<Engine> CreateEngine(const std::string& model_path,
                                                const std::string& backend_id = "onnx",
                                                std::string* error_message = nullptr);

    /** @brief True when @p backend_id is registered (`"onnx"` / `"tensorrt"`) */
    static bool HasBackend(const std::string& backend_id);

    /**
     * @brief Loads or reloads a model from a file path.
     *
     * Uses the backend instance already held by this engine (the one chosen
     * at creation time). Does not change the backend type.
     *
     * @param model_path Backend-specific model path.
     * @return True on success; false on failure (see @ref GetLastError).
     */
    bool LoadFromFile(const std::string& model_path);

    /**
     * @brief Loads or reloads a model using @ref InferenceOptions
     *
     * @param opt Model path and backend-specific session options
     * @return True on success; false on failure (see @ref GetLastError)
     */
    bool LoadFromOptions(const InferenceOptions& opt);

    /**
     * @brief Returns whether a model is currently loaded on the backend.
     *
     * @return False if there is no backend or the backend reports unloaded.
     */
    bool IsLoaded() const;

    /**
     * @brief Returns metadata for each model input tensor.
     *
     * Each entry includes the tensor name, @ref TensorShape, and
     * @ref ElementType. Empty if no backend is attached or the model is not
     * loaded.
     *
     * @return Ordered list of input tensor descriptors.
     */
    std::vector<ModelTensorInfo> GetInputInfos() const;

    /**
     * @brief Returns metadata for each model output tensor.
     *
     * @return Ordered list of output tensor descriptors.
     */
    std::vector<ModelTensorInfo> GetOutputInfos() const;

    /**
     * @brief Runs one forward pass with typed tensors.
     *
     * Input element types must match @ref GetInputInfos. Outputs use the
     * model's declared element types (see @ref Tensor).
     */
    bool Run(const TensorMap& inputs, TensorMap* outputs);

    /**
     * @brief Runs inference using float32 maps (converts to/from @ref TensorMap).
     */
    bool Run(const FloatTensorMap& inputs, FloatTensorMap* outputs);

    /**
     * @brief Runs one warmup forward pass (zero-filled inputs, static shapes only)
     *
     * Useful to pay session/provider initialization cost before measuring latency.
     * Skips inputs whose shapes are not fully static.
     *
     * @param[out] error Optional failure message
     * @return true when warmup ran or there was nothing to warm up
     */
    bool Warmup(std::string* error = nullptr);

    /**
     * @brief Returns the most recent error message for this engine.
     *
     * Prefers an engine-level message set by @ref Engine wrapper methods;
     * otherwise falls back to the owned backend's `GetLastError`. Returns a
     * reference to an empty string if there is no backend and no local error.
     *
     * @return Human-readable error text.
     */
    const std::string& GetLastError() const;

    /** @brief Factory id of the active backend (`"onnx"` or `"tensorrt"`) */
    const std::string& backend_id() const { return backend_id_; }

    /**
     * @brief Returns a non-owning pointer to the underlying backend.
     *
     * Intended for advanced extension or debugging. The pointer may be null
     * if the engine was default-constructed and never assigned a backend.
     *
     * @return Raw pointer to @ref Backend, or nullptr.
     */
    const Backend* backend() const {
        return backend_.get();
    }

private:
    explicit Engine(std::unique_ptr<Backend> backend, std::string backend_id);

    std::unique_ptr<Backend> backend_;  //!< @brief Owned inference backend
    std::string backend_id_;            //!< @brief "onnx" or "tensorrt"
    mutable std::string last_error_;    //!< @brief Last engine-level error text
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_BACKEND_ENGINE_HPP_
