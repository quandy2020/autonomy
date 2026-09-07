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

#include "autodriver/radar/backend_registry.hpp"

#include "autolink/common/log.hpp"

namespace autodriver {
namespace radar {

RadarBackendRegistry& RadarBackendRegistry::Instance() {
    static RadarBackendRegistry instance;
    return instance;
}

void RadarBackendRegistry::Register(const std::string& name,
                                    RadarDriverFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    factories_[name] = std::move(factory);
}

void RadarBackendRegistry::RegisterAlias(const std::string& alias,
                                         const std::string& canonical) {
    std::lock_guard<std::mutex> lock(mutex_);
    aliases_[alias] = canonical;
}

std::string RadarBackendRegistry::Resolve(const std::string& backend) const {
    const auto it = aliases_.find(backend);
    return it == aliases_.end() ? backend : it->second;
}

std::shared_ptr<SensorDriver> RadarBackendRegistry::Create(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string name = Resolve(backend.empty() ? "conti" : backend);
    const auto it = factories_.find(name);
    if (it == factories_.end()) {
        AERROR << "unknown radar backend: " << backend;
        return nullptr;
    }
    return it->second(id, params);
}

bool RadarBackendRegistry::Has(const std::string& backend) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.count(Resolve(backend)) > 0;
}

void RegisterRadarBackendWithAliases(
    const std::string& name, RadarDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    auto& reg = RadarBackendRegistry::Instance();
    reg.Register(name, std::move(factory));
    for (const char* alias : aliases) {
        if (alias && *alias) {
            reg.RegisterAlias(alias, name);
        }
    }
}

}  // namespace radar
}  // namespace autodriver
