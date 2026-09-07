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

#include "autodriver/camera/backend_registry.hpp"

#include <utility>

#include "autolink/common/log.hpp"

namespace autodriver {
namespace camera {
namespace {

template <typename Registry>
std::string ResolveLocked(
    const Registry& /*unused*/,
    const std::unordered_map<std::string, std::string>& aliases,
    const std::string& backend) {
    const auto it = aliases.find(backend);
    if (it != aliases.end()) {
        return it->second;
    }
    return backend;
}

}  // namespace

CameraBackendRegistry& CameraBackendRegistry::Instance() {
    static CameraBackendRegistry registry;
    return registry;
}

void CameraBackendRegistry::Register(const std::string& name,
                                     CameraDriverFactory factory) {
    if (name.empty() || !factory) {
        AERROR << "CameraBackendRegistry::Register: empty name or factory";
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (factories_.count(name) != 0) {
        AWARN << "CameraBackendRegistry: overwriting backend \"" << name
              << "\"";
    }
    factories_[name] = std::move(factory);
}

void CameraBackendRegistry::RegisterAlias(const std::string& alias,
                                          const std::string& canonical) {
    if (alias.empty() || canonical.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    aliases_[alias] = canonical;
}

std::string CameraBackendRegistry::Resolve(const std::string& backend) const {
    return ResolveLocked(*this, aliases_, backend);
}

bool CameraBackendRegistry::Has(const std::string& backend) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.count(Resolve(backend)) != 0;
}

std::shared_ptr<SensorDriver> CameraBackendRegistry::Create(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
    CameraDriverFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = factories_.find(Resolve(backend));
        if (it == factories_.end()) {
            AERROR << "unsupported camera backend: " << backend;
            return nullptr;
        }
        factory = it->second;
    }
    return factory(id, params);
}

PointCloudBackendRegistry& PointCloudBackendRegistry::Instance() {
    static PointCloudBackendRegistry registry;
    return registry;
}

void PointCloudBackendRegistry::Register(const std::string& name,
                                         PointCloudDriverFactory factory) {
    if (name.empty() || !factory) {
        AERROR << "PointCloudBackendRegistry::Register: empty name or factory";
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (factories_.count(name) != 0) {
        AWARN << "PointCloudBackendRegistry: overwriting backend \"" << name
              << "\"";
    }
    factories_[name] = std::move(factory);
}

void PointCloudBackendRegistry::RegisterAlias(const std::string& alias,
                                              const std::string& canonical) {
    if (alias.empty() || canonical.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    aliases_[alias] = canonical;
}

std::string PointCloudBackendRegistry::Resolve(
    const std::string& backend) const {
    return ResolveLocked(*this, aliases_, backend);
}

bool PointCloudBackendRegistry::Has(const std::string& backend) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.count(Resolve(backend)) != 0;
}

std::shared_ptr<SensorDriver> PointCloudBackendRegistry::Create(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
    PointCloudDriverFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = factories_.find(Resolve(backend));
        if (it == factories_.end()) {
            AERROR << "unsupported point cloud backend: " << backend;
            return nullptr;
        }
        factory = it->second;
    }
    return factory(id, params);
}

void RegisterCameraBackendWithAliases(
    const std::string& name, CameraDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    auto& reg = CameraBackendRegistry::Instance();
    reg.Register(name, std::move(factory));
    for (const char* alias : aliases) {
        if (alias != nullptr && alias[0] != '\0') {
            reg.RegisterAlias(alias, name);
        }
    }
}

void RegisterPointCloudBackendWithAliases(
    const std::string& name, PointCloudDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    auto& reg = PointCloudBackendRegistry::Instance();
    reg.Register(name, std::move(factory));
    for (const char* alias : aliases) {
        if (alias != nullptr && alias[0] != '\0') {
            reg.RegisterAlias(alias, name);
        }
    }
}

}  // namespace camera
}  // namespace autodriver
