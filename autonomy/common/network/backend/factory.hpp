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

#ifndef AUTONOMY_COMMON_NETWORK_BACKEND_FACTORY_HPP_
#define AUTONOMY_COMMON_NETWORK_BACKEND_FACTORY_HPP_

#include <memory>
#include <string>

#include "autonomy/common/network/backend/backend.hpp"
#include "autonomy/common/network/backend/registry.hpp"
#include "autonomy/common/network/common/options.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file factory.hpp
 * @brief Process-wide backend registry and model loading (library-internal)
 *
 * Wraps `autonomy::common::Factory` string id → @ref Backend creator.
 * Application code should use @ref Engine. Formerly named `Inference`.
 *
 * Built-in ids:
 * - `"onnx"` — @ref OnnxBackend
 * - `"tensorrt"` — @ref TensorRtBackend (when compiled in)
 */

/**
 * @brief Static factory: register, query, and create @ref Backend instances
 */
class BackendFactory
{
public:
    using Registry = NetworkBackendFactory;  //!< String-id factory type

    /**
     * @brief Returns the process-wide singleton registry
     * @note @ref EnsureBuiltinsRegistered runs before first Create
     */
    static Registry& RegistryInstance();

    /** @brief Thread-safe one-time registration of built-in backends */
    static void EnsureBuiltinsRegistered();

    /**
     * @brief Whether @p id is registered
     * @param id e.g. `"onnx"`, `"tensorrt"`
     */
    static bool HasBackend(const std::string& id);

    /**
     * @brief Creates a backend and loads a model
     *
     * @param opt backend_id, model_path, and nested options
     * @param[out] error_message Failure reason; may be nullptr
     * @return Backend after successful LoadFromOptions, or nullptr
     */
    static std::unique_ptr<Backend> Create(
        const InferenceOptions& opt, std::string* error_message = nullptr);

    /**
     * @brief Registers a custom backend creator
     * @param id Unique string id
     * @param creator Returns heap-allocated @ref Backend
     */
    static bool Register(const std::string& id, Backend* (*creator)()) {
        return RegistryInstance().Register(id, creator);
    }
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_BACKEND_FACTORY_HPP_
