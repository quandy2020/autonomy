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

#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {

LidarBackendRegistry& LidarBackendRegistry::Instance() {
  static LidarBackendRegistry registry;
  return registry;
}

void LidarBackendRegistry::RegisterBackend(const std::string& name,
                                           LidarDriverFactory factory) {
  factory_.Register(name, std::move(factory));
}

void LidarBackendRegistry::RegisterBackendAlias(const std::string& alias,
                                                const std::string& canonical) {
  factory_.RegisterAlias(alias, canonical);
}

SensorDriver::SharedPtr LidarBackendRegistry::CreateDriver(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
  const std::string name = backend.empty() ? "velodyne" : backend;
  auto driver = factory_.CreateShared(name, id, params);
  if (!driver) {
    AERROR << "unsupported lidar3d backend: " << name;
  }
  return driver;
}

bool LidarBackendRegistry::HasBackend(const std::string& backend) const {
  return factory_.Contains(backend.empty() ? "velodyne" : backend);
}

void RegisterLidarBackendWithAliases(
    const std::string& name, LidarDriverFactory factory,
    std::initializer_list<const char*> aliases) {
  auto& reg = LidarBackendRegistry::Instance();
  reg.RegisterBackend(name, std::move(factory));
  for (const char* alias : aliases) {
    if (alias != nullptr && alias[0] != '\0') {
      reg.RegisterBackendAlias(alias, name);
    }
  }
}

}  // namespace lidar
}  // namespace autodriver
