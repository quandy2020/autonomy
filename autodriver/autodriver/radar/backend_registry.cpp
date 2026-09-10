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
  static RadarBackendRegistry registry;
  return registry;
}

void RadarBackendRegistry::RegisterBackend(const std::string& name,
    RadarDriverFactory factory) {
  factory_.Register(name, std::move(factory));
}

void RadarBackendRegistry::RegisterBackendAlias(const std::string& alias,
    const std::string& canonical) {
  factory_.RegisterAlias(alias, canonical);
}

SensorDriver::SharedPtr RadarBackendRegistry::CreateDriver(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
  const std::string name =
      backend.empty() ? "conti" : backend;
  auto driver = factory_.CreateShared(name, id, params);
  if (!driver) {
    AERROR << "unknown radar backend: " << backend;
  }
  return driver;
}

bool RadarBackendRegistry::HasBackend(const std::string& backend) const {
  return factory_.Contains(
      backend.empty() ? "conti" : backend);
}

void RegisterRadarBackendWithAliases(
    const std::string& name, RadarDriverFactory factory,
    std::initializer_list<const char*> aliases) {
  auto& reg = RadarBackendRegistry::Instance();
  reg.RegisterBackend(name, std::move(factory));
  for (const char* alias : aliases) {
    if (alias != nullptr && *alias != static_cast<char>(0)) {
      reg.RegisterBackendAlias(alias, name);
    }
  }
}

}  // namespace radar
}  // namespace autodriver
