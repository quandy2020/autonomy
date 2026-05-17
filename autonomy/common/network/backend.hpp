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

#ifndef AUTONOMY_COMMON_NETWORK_BACKEND_HPP_
#define AUTONOMY_COMMON_NETWORK_BACKEND_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/options.hpp"
#include "autonomy/common/network/registry.hpp"
#include "autonomy/common/network/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file backend.hpp
 * @brief Abstract inference backend interface (ONNX, TensorRT, RKNN, ...)
 */

/**
 * @brief Platform-specific model inference implementation
 *
 * Subclasses implement @ref LoadFromFile, @ref Run, and tensor metadata queries.
 * Usually owned by @ref Engine; created via @ref Inference and @ref NetworkBackendFactory.
 */
class Backend {
public:
    Backend() = default;

    /**
     * @brief Destroys the backend.
     */
    virtual ~Backend();

    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;

    /**
     * @brief Loads a model from a file path.
     *
     * The meaning of `model_path` is backend-specific (for example, a `.onnx`
     * file for ONNX Runtime).
     *
     * @param model_path Path to the model on disk.
     * @return True on success; false on failure (see @ref GetLastError).
     */
    virtual bool LoadFromFile(const std::string& model_path) = 0;

    /**
     * @brief Loads a model using @ref InferenceOptions.
     *
     * The default implementation in `backend.cpp` forwards to
     * `LoadFromFile(opt.model_path)`. Overrides may read `onnx`, `tensorrt`,
     * or `rknn` option slices.
     *
     * @param opt Unified configuration passed from @ref Engine or @ref Inference.
     * @return True on success; false on failure.
     */
    virtual bool LoadFromOptions(const InferenceOptions& opt);

    /**
     * @brief Returns whether a model is currently loaded.
     *
     * @return True if the backend holds a runnable session or engine.
     */
    virtual bool IsLoaded() const = 0;

    /**
     * @brief Returns metadata for each model input.
     *
     * @return Empty if not loaded; otherwise one @ref ModelTensorInfo per input.
     */
    virtual std::vector<ModelTensorInfo> GetInputInfos() const = 0;

    /**
     * @brief Returns metadata for each model output.
     *
     * @return Empty if not loaded; otherwise one @ref ModelTensorInfo per output.
     */
    virtual std::vector<ModelTensorInfo> GetOutputInfos() const = 0;

    /**
     * @brief Executes one forward pass.
     *
     * @param inputs Map from tensor name to contiguous float32 data (row-major).
     * @param outputs On success, cleared and filled with output tensors. Must
     *                not be null.
     * @return True on success; false on failure (see @ref GetLastError).
     */
    virtual bool Run(const std::unordered_map<std::string, std::vector<float>>& inputs,
                     std::unordered_map<std::string, std::vector<float>>* outputs) = 0;

    /**
     * @brief Returns the last error message set by this backend.
     *
     * @return Reference to the stored error string (may be empty).
     */
    const std::string& GetLastError() const {
        return last_error_;
    }

protected:
    /**
     * @brief Records an error message for @ref GetLastError.
     *
     * @param msg Error text (moved into storage).
     */
    void SetLastError(std::string msg) {
        last_error_ = std::move(msg);
    }

    /**
     * @brief Clears the stored error message.
     */
    void ClearLastError() {
        last_error_.clear();
    }

    std::string last_error_;  //!< @brief Last error set via @ref SetLastError
};

/**
 * @brief Registers backends enabled at CMake configure time.
 *
 * Always registers `"onnx"`. Optionally registers `"tensorrt"` and `"rknn"`
 * when `AUTONOMY_BUILD_NETWORK_TENSORRT` / `AUTONOMY_BUILD_NETWORK_RKNN` are
 * defined on the autonomy target.
 *
 * @param factory Factory instance from @ref Inference::BackendFactoryInstance.
 * @return True if all attempted registrations succeeded.
 */
bool RegisterBuiltinNetworkBackends(NetworkBackendFactory& factory);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_BACKEND_HPP_
