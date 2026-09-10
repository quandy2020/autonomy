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

#include "autodriver/lidar/lidar_2d_backend_registry.hpp"

#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {

Lidar2dBackendRegistry& Lidar2dBackendRegistry::Instance() {
  static Lidar2dBackendRegistry registry;
  return registry;
}

void Lidar2dBackendRegistry::RegisterBackend(const std::string& name,
    Lidar2dDriverFactory factory) {
  factory_.Register(name, std::move(factory));
}

void Lidar2dBackendRegistry::RegisterBackendAlias(const std::string& alias,
    const std::string& canonical) {
  factory_.RegisterAlias(alias, canonical);
}

SensorDriver::SharedPtr Lidar2dBackendRegistry::CreateDriver(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
  const std::string name = backend.empty() ? "rplidar" : backend;
  auto driver = factory_.CreateShared(name, id, params);
  if (!driver) {
    AERROR << "unsupported lidar2d backend: " << name;
  }
  return driver;
}

bool Lidar2dBackendRegistry::HasBackend(const std::string& backend) const {
  return factory_.Contains(backend.empty() ? "rplidar" : backend);
}

void RegisterLidar2dBackendWithAliases(
    const std::string& name, Lidar2dDriverFactory factory,
    std::initializer_list<const char*> aliases) {
  auto& reg = Lidar2dBackendRegistry::Instance();
  reg.RegisterBackend(name, std::move(factory));
  for (const char* alias : aliases) {
    if (alias != nullptr && *alias != static_cast<char>(0)) {
      reg.RegisterBackendAlias(alias, name);
    }
  }
}

}  // namespace lidar
}  // namespace autodriver
