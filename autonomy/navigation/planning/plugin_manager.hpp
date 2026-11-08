/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file plugin_manager.hpp
 * @brief Static plugin registry for encoder, policy and memory factories.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "navigation/memory/context_provider.hpp"
#include "navigation/perception/observation_encoder.hpp"
#include "navigation/planning/interface/policy_interface.hpp"
#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace planning
{

class PluginManager
{
public:
    /** @brief Factory for an ObservationEncoder. */
    using EncoderFactory = std::function<std::unique_ptr<perception::ObservationEncoder>(
        const config::NavigationConfig&)>;
    /** @brief Factory for a policy plugin. */
    using PolicyFactory = std::function<std::unique_ptr<PolicyInterface>(
        const config::NavigationConfig&)>;
    /** @brief Factory for a memory provider. */
    using MemoryFactory = std::function<std::unique_ptr<memory::ContextProvider>(
        const config::MemoryOptions&)>;

    /** @brief Process-wide registry. */
    static PluginManager& Instance();

    /**
     * @brief Register an encoder factory.
     *
     * @param[in] name Configuration name, e.g. "defm".
     * @param[in] factory Callable creating the encoder.
     */
    void RegisterEncoder(const std::string& name, EncoderFactory factory);

    /**
     * @brief Register a policy factory.
     *
     * @param[in] name Configuration name, e.g. "vntpa".
     * @param[in] factory Callable creating the policy.
     */
    void RegisterPolicy(const std::string& name, PolicyFactory factory);

    /**
     * @brief Register a memory factory.
     *
     * @param[in] name Configuration name, e.g. "pose_indexed".
     * @param[in] factory Callable creating the provider.
     */
    void RegisterMemory(const std::string& name, MemoryFactory factory);

    /**
     * @brief Create an encoder by name.
     *
     * @param[in] name Registered factory name.
     * @param[in] configuration Full configuration.
     * @return Encoder, or nullptr when the name is unknown.
     */
    std::unique_ptr<perception::ObservationEncoder> CreateEncoder(
        const std::string& name, const config::NavigationConfig& configuration);

    /**
     * @brief Create a policy by name.
     *
     * @param[in] name Registered factory name.
     * @param[in] configuration Full configuration.
     * @return Policy, or nullptr when the name is unknown.
     */
    std::unique_ptr<PolicyInterface> CreatePolicy(
        const std::string& name, const config::NavigationConfig& configuration);

    /**
     * @brief Create a memory provider by name.
     *
     * @param[in] name Registered factory name.
     * @param[in] options Memory options.
     * @return Provider, or nullptr when the name is unknown.
     */
    std::unique_ptr<memory::ContextProvider> CreateMemory(
        const std::string& name, const config::MemoryOptions& options);

    /** @brief Test-only: drop every registered factory. */
    void Clear();

private:
    PluginManager() = default;

    /** @brief Protects the three maps. */
    std::mutex mutex_;
    /** @brief Encoder factories by name. */
    std::unordered_map<std::string, EncoderFactory> encoders_;
    /** @brief Policy factories by name. */
    std::unordered_map<std::string, PolicyFactory> policies_;
    /** @brief Memory factories by name. */
    std::unordered_map<std::string, MemoryFactory> memories_;
};

}  // namespace planning
}  // namespace navigation
