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

#ifndef AUTONOMY_COMMON_NETWORK_INFERENCE_HPP_
#define AUTONOMY_COMMON_NETWORK_INFERENCE_HPP_

#include <memory>
#include <string>

#include "autonomy/common/network/backend.hpp"
#include "autonomy/common/network/options.hpp"
#include "autonomy/common/network/registry.hpp"

namespace autonomy {
namespace common {
namespace network {

/**
 * @file inference.hpp
 * @brief Backend factory registry and custom backend registration
 */

/**
 * @brief Owns the process-wide @ref NetworkBackendFactory
 *
 * Prefer @ref Engine for application inference. Use this class to register
 * custom backends or create @ref Backend instances directly.
 */
class Inference {
public:
    using BackendFactory = NetworkBackendFactory;

    /**
     * @brief Returns the process-wide backend factory singleton.
     *
     * @return Reference to the factory map (id -> creator function).
     */
    static BackendFactory& BackendFactoryInstance();

    /**
     * @brief Registers built-in backends exactly once (thread-safe).
     */
    static void EnsureBuiltinBackendsRegistered();

    /**
     * @brief Returns true if `id` is registered in the factory.
     *
     * @param id Backend factory key (e.g. `"onnx"`).
     */
    static bool HasBackend(const std::string& id);

    /**
     * @brief Creates a backend and loads a model from options.
     *
     * @param opt Configuration including backend id and model path.
     * @param error_message Optional failure reason when nullptr is returned.
     * @return Loaded backend on success, or nullptr on failure.
     */
    static std::unique_ptr<Backend> CreateBackend(
        const InferenceOptions& opt, std::string* error_message = nullptr);

    /**
     * @brief Registers a custom backend creator under `id`.
     *
     * @param id Factory key used in @ref InferenceOptions::backend_id.
     * @param creator Function that returns a new @ref Backend instance.
     * @return True if `id` was not already registered.
     */
    static bool RegisterBackend(const std::string& id,
                                Backend* (*creator)()) {
        return BackendFactoryInstance().Register(id, creator);
    }
};

}  // namespace network
}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_NETWORK_INFERENCE_HPP_
