/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/lidar/backend_registry.hpp"

#include <utility>

#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {

LidarBackendRegistry& LidarBackendRegistry::Instance() {
    static LidarBackendRegistry registry;
    return registry;
}

void LidarBackendRegistry::Register(const std::string& name,
                                    LidarDriverFactory factory) {
    if (name.empty() || !factory) {
        AERROR << "LidarBackendRegistry::Register: empty name or factory";
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (factories_.count(name) != 0) {
        AWARN << "LidarBackendRegistry: overwriting backend \"" << name << "\"";
    }
    factories_[name] = std::move(factory);
}

void LidarBackendRegistry::RegisterAlias(const std::string& alias,
                                         const std::string& canonical) {
    if (alias.empty() || canonical.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    aliases_[alias] = canonical;
}

std::string LidarBackendRegistry::Resolve(const std::string& backend) const {
    const auto it = aliases_.find(backend);
    if (it != aliases_.end()) {
        return it->second;
    }
    return backend;
}

bool LidarBackendRegistry::Has(const std::string& backend) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string name = Resolve(backend);
    return factories_.count(name) != 0;
}

std::shared_ptr<SensorDriver> LidarBackendRegistry::Create(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
    LidarDriverFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string name = Resolve(backend);
        const auto it = factories_.find(name);
        if (it == factories_.end()) {
            AERROR << "unsupported lidar3d backend: " << backend;
            return nullptr;
        }
        factory = it->second;
    }
    return factory(id, params);
}

void RegisterLidarBackendWithAliases(
    const std::string& name, LidarDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    auto& reg = LidarBackendRegistry::Instance();
    reg.Register(name, std::move(factory));
    for (const char* alias : aliases) {
        if (alias != nullptr && alias[0] != '\0') {
            reg.RegisterAlias(alias, name);
        }
    }
}

}  // namespace lidar
}  // namespace autodriver
