/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

/**
 * @file backend_registry.cpp
 * @brief Arm backend registry (implementation).
 */

#include "automanip/arm/backend_registry.hpp"

#include "autolink/common/log.hpp"

namespace automanip {
namespace arm {

ArmBackendRegistry& ArmBackendRegistry::Instance() {
  static ArmBackendRegistry registry;
  return registry;
}

void ArmBackendRegistry::Register(const std::string& name,
                                  ArmDriverFactory factory) {
  std::lock_guard<std::mutex> lock(mutex_);
  factories_[name] = factory;
}

void ArmBackendRegistry::RegisterAlias(const std::string& alias,
                                       const std::string& canonical) {
  std::lock_guard<std::mutex> lock(mutex_);
  aliases_[alias] = canonical;
}

std::shared_ptr<ArmDriver> ArmBackendRegistry::Create(
    const std::string& backend, const std::string& id,
    const ArmPlantOptions& options) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (backend.empty()) {
    AERROR << "arm backend is empty";
    return nullptr;
  }
  std::string name = backend;
  const auto alias = aliases_.find(name);
  if (alias != aliases_.end()) {
    name = alias->second;
  }
  const auto factory = factories_.find(name);
  if (factory == factories_.end() || factory->second == nullptr) {
    AERROR << "unknown arm backend: " << backend;
    return nullptr;
  }
  return std::shared_ptr<ArmDriver>(factory->second(id, options));
}

}  // namespace arm
}  // namespace automanip
