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

#include "autolink/common/log.hpp"

namespace autodriver {
namespace camera {
namespace {

template <typename Registry>
void RegisterAliases(Registry& reg, const std::string& name,
                     std::initializer_list<const char*> aliases) {
  for (const char* alias : aliases) {
    if (alias != nullptr && *alias != static_cast<char>(0)) {
      reg.RegisterBackendAlias(alias, name);
    }
  }
}

}  // namespace

CameraBackendRegistry& CameraBackendRegistry::Instance() {
  static CameraBackendRegistry registry;
  return registry;
}

void CameraBackendRegistry::RegisterBackend(const std::string& name,
                                            CameraDriverFactory factory) {
  factory_.Register(name, std::move(factory));
}

void CameraBackendRegistry::RegisterBackendAlias(
    const std::string& alias, const std::string& canonical) {
  factory_.RegisterAlias(alias, canonical);
}

SensorDriver::SharedPtr CameraBackendRegistry::CreateDriver(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
  const std::string name = backend.empty() ? "realsense" : backend;
  auto driver = factory_.CreateShared(name, id, params);
  if (!driver) {
    AERROR << "unsupported camera backend: " << name;
  }
  return driver;
}

bool CameraBackendRegistry::HasBackend(const std::string& backend) const {
  return factory_.Contains(backend.empty() ? "realsense" : backend);
}

PointCloudBackendRegistry& PointCloudBackendRegistry::Instance() {
  static PointCloudBackendRegistry registry;
  return registry;
}

void PointCloudBackendRegistry::RegisterBackend(
    const std::string& name, PointCloudDriverFactory factory) {
  factory_.Register(name, std::move(factory));
}

void PointCloudBackendRegistry::RegisterBackendAlias(
    const std::string& alias, const std::string& canonical) {
  factory_.RegisterAlias(alias, canonical);
}

SensorDriver::SharedPtr PointCloudBackendRegistry::CreateDriver(
    const std::string& backend, const SensorId& id,
    const hardware::DriverParams& params) const {
  const std::string name = backend.empty() ? "realsense" : backend;
  auto driver = factory_.CreateShared(name, id, params);
  if (!driver) {
    AERROR << "unsupported point cloud backend: " << name;
  }
  return driver;
}

bool PointCloudBackendRegistry::HasBackend(const std::string& backend) const {
  return factory_.Contains(backend.empty() ? "realsense" : backend);
}

void RegisterCameraBackendWithAliases(
    const std::string& name, CameraDriverFactory factory,
    std::initializer_list<const char*> aliases) {
  auto& reg = CameraBackendRegistry::Instance();
  reg.RegisterBackend(name, std::move(factory));
  RegisterAliases(reg, name, aliases);
}

void RegisterPointCloudBackendWithAliases(
    const std::string& name, PointCloudDriverFactory factory,
    std::initializer_list<const char*> aliases) {
  auto& reg = PointCloudBackendRegistry::Instance();
  reg.RegisterBackend(name, std::move(factory));
  RegisterAliases(reg, name, aliases);
}

}  // namespace camera
}  // namespace autodriver
