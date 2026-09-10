/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/lidar/lidar_2d_backend_registry.hpp"

#include <utility>

#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {

Lidar2dBackendRegistry& Lidar2dBackendRegistry::Instance() {
    static Lidar2dBackendRegistry registry;
    return registry;
}

void Lidar2dBackendRegistry::Register(const std::string& name,
                                      Lidar2dDriverFactory factory) {
    if (name.empty() || !factory) {
        AERROR << "Lidar2dBackendRegistry::Register: empty name or factory";
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (factories_.count(name) != 0) {
        AWARN << "Lidar2dBackendRegistry: overwriting backend \"" << name
              << "\"";
    }
    factories_[name] = std::move(factory);
}

void Lidar2dBackendRegistry::RegisterAlias(const std::string& alias,
                                           const std::string& canonical) {
    if (alias.empty() || canonical.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    aliases_[alias] = canonical;
}

std::string Lidar2dBackendRegistry::Resolve(const std::string& backend) const {
    const auto it = aliases_.find(backend);
    if (it != aliases_.end()) {
        return it->second;
    }
    return backend;
}

bool Lidar2dBackendRegistry::Has(const std::string& backend) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.count(Resolve(backend)) != 0;
}

std::shared_ptr<SensorDriver> Lidar2dBackendRegistry::Create(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
    Lidar2dDriverFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string name = Resolve(backend);
        const auto it = factories_.find(name);
        if (it == factories_.end()) {
            AERROR << "unsupported lidar2d backend: " << backend;
            return nullptr;
        }
        factory = it->second;
    }
    return factory(id, params);
}

void RegisterLidar2dBackendWithAliases(
    const std::string& name, Lidar2dDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    auto& reg = Lidar2dBackendRegistry::Instance();
    reg.Register(name, std::move(factory));
    for (const char* alias : aliases) {
        if (alias != nullptr && alias[0] != '\0') {
            reg.RegisterAlias(alias, name);
        }
    }
}

}  // namespace lidar
}  // namespace autodriver
