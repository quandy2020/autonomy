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

#include "chassis/backend_registry.hpp"

#include "autolink/common/log.hpp"

namespace autodriver {
namespace chassis {

ChassisBackendRegistry& ChassisBackendRegistry::Instance() {
  static ChassisBackendRegistry instance;
  return instance;
}

void ChassisBackendRegistry::RegisterBackend(const std::string& name,
                                             ChassisDriverFactory factory) {
  factory_.Register(name, std::move(factory));
}

void ChassisBackendRegistry::RegisterBackendAlias(
    const std::string& alias, const std::string& canonical) {
  factory_.RegisterAlias(alias, canonical);
}

ChassisDriver::SharedPtr ChassisBackendRegistry::CreateDriver(
    const std::string& backend, const ChassisId& id,
    const hardware::DriverParams& params) const {
  const std::string name = backend.empty() ? "stub" : backend;
  auto driver = factory_.CreateShared(name, id, params);
  if (!driver) {
    AERROR << "unknown chassis backend: " << backend;
  }
  return driver;
}

bool ChassisBackendRegistry::HasBackend(const std::string& backend) const {
  return factory_.Contains(backend.empty() ? "stub" : backend);
}

void RegisterChassisBackendWithAliases(
    const std::string& name, ChassisDriverFactory factory,
    std::initializer_list<const char*> aliases) {
  auto& reg = ChassisBackendRegistry::Instance();
  reg.RegisterBackend(name, std::move(factory));
  for (const char* alias : aliases) {
    if (alias && *alias) {
      reg.RegisterBackendAlias(alias, name);
    }
  }
}

}  // namespace chassis
}  // namespace autodriver
