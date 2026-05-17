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

#ifndef AUTONOMY_COMMON_NETWORK_ENGINE_HPP_
#define AUTONOMY_COMMON_NETWORK_ENGINE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/options.hpp"
#include "autonomy/common/network/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {

class Backend;

/**
 * @file engine.hpp
 * @brief Public entry point for neural network model inference
 */

/**
 * @brief Inference engine wrapping a platform-specific @ref Backend
 *
 * Loads a model, exposes input/output @ref ModelTensorInfo, and runs forward
 * passes with named float32 @ref TensorMap buffers. Does not perform image
 * preprocessing; use @ref RunPipeline in process.hpp when needed.
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
     * @brief Creates an engine from full inference options.
     *
     * Selects a backend using `opt.backend_id` (for example `"onnx"`,
     * `"tensorrt"`, or `"rknn"`), instantiates it through the factory
     * registered by @ref Inference, and loads the model from `opt.model_path`
     * (and backend-specific fields inside `opt`).
     *
     * @param opt Unified configuration (backend id, model path, ORT/TRT/RKNN
     *            slices). See @ref InferenceOptions.
     * @param error_message If non-null, filled with a failure reason when
     *                      this function returns nullptr.
     * @return A loaded engine on success, or nullptr on failure.
     */
    static std::unique_ptr<Engine> CreateEngine(
        const InferenceOptions& opt, std::string* error_message = nullptr);

    /**
     * @brief Creates an engine from a model path and backend identifier.
     *
     * Convenience wrapper that builds an @ref InferenceOptions with
     * `model_path` and `backend_id`, then calls
     * `CreateEngine(opt, error_message)`.
     *
     * @param model_path Path to the model file (format depends on the
     *                   backend, e.g. `.onnx` for ONNX Runtime).
     * @param backend_id Factory key for the backend implementation.
     * @param error_message If non-null, filled when creation fails.
     * @return A loaded engine on success, or nullptr on failure.
     */
    static std::unique_ptr<Engine> CreateEngine(
        const std::string& model_path, const std::string& backend_id = "onnx",
        std::string* error_message = nullptr);

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
     * @brief Loads or reloads a model using @ref InferenceOptions.
     *
     * If `opt.backend_id` is non-empty, updates the cached backend id string
     * returned by @ref backend_id. Forwards remaining fields to the owned
     * backend's `LoadFromOptions`.
     *
     * @param opt Configuration including model path and backend options.
     * @return True on success; false on failure (see @ref GetLastError).
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
     * @brief Runs one forward pass.
     *
     * Keys in `inputs` must match input tensor names from @ref GetInputInfos.
     * Each value is a contiguous `float` buffer in row-major layout with an
     * element count consistent with the model shape (including resolved
     * dynamic dimensions where supported by the backend). On success,
     * `outputs` is cleared and repopulated with output names and buffers.
     *
     * @param inputs Map from input name to float data.
     * @param outputs On success, receives output name to float data. Must not
     *                be null.
     * @return True on success; false on failure (see @ref GetLastError).
     */
    bool Run(const std::unordered_map<std::string, std::vector<float>>& inputs,
             std::unordered_map<std::string, std::vector<float>>* outputs);

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

    /**
     * @brief Returns the backend factory id used by this engine.
     *
     * Examples: `"onnx"`, `"tensorrt"`, `"rknn"`. Updated when
     * @ref LoadFromOptions is called with a non-empty `backend_id`.
     *
     * @return Backend identifier string.
     */
    const std::string& backend_id() const {
        return backend_id_;
    }

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
    /**
     * @brief Constructs an engine that already owns a loaded backend.
     *
     * @param backend Transferred backend instance.
     * @param backend_id Factory id associated with `backend`.
     */
    explicit Engine(std::unique_ptr<Backend> backend, std::string backend_id);

    std::unique_ptr<Backend> backend_;  //!< @brief Owned runtime backend instance
    std::string backend_id_;            //!< @brief Factory key, e.g. "onnx"
    mutable std::string last_error_;    //!< @brief Last engine-level error text
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_ENGINE_HPP_
