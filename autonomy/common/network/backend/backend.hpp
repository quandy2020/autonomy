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

#ifndef AUTONOMY_COMMON_NETWORK_BACKEND_BACKEND_HPP_
#define AUTONOMY_COMMON_NETWORK_BACKEND_BACKEND_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/network/backend/registry.hpp"
#include "autonomy/common/network/common/options.hpp"
#include "autonomy/common/network/common/tensor.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file backend.hpp
 * @brief Abstract inference backend (ONNX Runtime, TensorRT, …)
 *
 * Application code should use @ref Engine. This interface is for new backend
 * implementations and factory registration. Built-ins register via
 * @ref RegisterBuiltinNetworkBackends into @ref BackendFactory.
 */

/**
 * @brief Abstract backend: one model load and forward pass API
 *
 * Implementations must populate `GetLastError()` after failed calls.
 *
 * @note Not thread-safe unless documented otherwise for a specific backend.
 *       Assume single-threaded @ref Run unless externally synchronized.
 */
class Backend
{
public:
    Backend() = default;
    virtual ~Backend();

    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;
    Backend(Backend&&) noexcept = default;
    Backend& operator=(Backend&&) noexcept = default;

    /**
     * @brief Loads a model from a file path
     * @param model_path Backend-specific extension (.onnx, .engine, …)
     * @return true on success
     */
    virtual bool LoadFromFile(const std::string& model_path) = 0;

    /**
     * @brief Loads from @ref InferenceOptions (default:
     * LoadFromFile(model_path))
     * @param opt Path and backend-specific options
     */
    virtual bool LoadFromOptions(const InferenceOptions& opt);

    /** @return true when a model is loaded */
    virtual bool IsLoaded() const = 0;

    /** @return Input tensor metadata (valid after load) */
    virtual std::vector<ModelTensorInfo> GetInputInfos() const = 0;

    /** @return Output tensor metadata */
    virtual std::vector<ModelTensorInfo> GetOutputInfos() const = 0;

    /**
     * @brief Runs one forward pass
     *
     * @param inputs Keys are input names; element types must match @ref
     * GetInputInfos
     * @param[out] outputs Filled by the implementation
     * @return true on success
     */
    virtual bool Run(const TensorMap& inputs, TensorMap* outputs) = 0;

    /**
     * @brief float32 convenience Run (converts to/from @ref TensorMap
     * internally)
     * @param inputs Float input map
     * @param[out] outputs Float output map
     */
    bool Run(const FloatTensorMap& inputs, FloatTensorMap* outputs);

    /** @return Last error description (may be empty) */
    const std::string& GetLastError() const {
        return last_error_;
    }

protected:
    /** @brief Sets error message (overwrites previous) */
    void SetLastError(std::string msg) {
        last_error_ = std::move(msg);
    }

    /** @brief Clears error message */
    void ClearLastError() {
        last_error_.clear();
    }

    std::string last_error_;  //!< Backend-level error cache
};

/**
 * @brief Registers built-in backends (onnx, tensorrt, …) into the factory
 *
 * @param factory Usually @ref BackendFactory::RegistryInstance()
 * @return true when all registrations succeed
 */
bool RegisterBuiltinNetworkBackends(NetworkBackendFactory& factory);

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_BACKEND_BACKEND_HPP_
