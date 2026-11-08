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

#include "navigation/planning/plugin_manager.hpp"

#include <utility>

namespace navigation
{
namespace planning
{

PluginManager& PluginManager::Instance()
{
    static PluginManager instance;
    return instance;
}

void PluginManager::RegisterEncoder(const std::string& name, EncoderFactory factory)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    encoders_[name] = std::move(factory);
}

void PluginManager::RegisterPolicy(const std::string& name, PolicyFactory factory)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    policies_[name] = std::move(factory);
}

void PluginManager::RegisterMemory(const std::string& name, MemoryFactory factory)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    memories_[name] = std::move(factory);
}

std::unique_ptr<perception::ObservationEncoder> PluginManager::CreateEncoder(
    const std::string& name, const config::NavigationConfig& configuration)
{
    EncoderFactory factory;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto iterator = encoders_.find(name);
        if (iterator == encoders_.end())
        {
            return nullptr;
        }
        factory = iterator->second;
    }
    return factory(configuration);
}

std::unique_ptr<PolicyInterface> PluginManager::CreatePolicy(
    const std::string& name, const config::NavigationConfig& configuration)
{
    PolicyFactory factory;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto iterator = policies_.find(name);
        if (iterator == policies_.end())
        {
            return nullptr;
        }
        factory = iterator->second;
    }
    return factory(configuration);
}

std::unique_ptr<memory::ContextProvider> PluginManager::CreateMemory(
    const std::string& name, const config::MemoryOptions& options)
{
    MemoryFactory factory;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto iterator = memories_.find(name);
        if (iterator == memories_.end())
        {
            return nullptr;
        }
        factory = iterator->second;
    }
    return factory(options);
}

void PluginManager::Clear()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    encoders_.clear();
    policies_.clear();
    memories_.clear();
}

}  // namespace planning
}  // namespace navigation
