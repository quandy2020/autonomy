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
 * @file backend_registry.hpp
 * @brief Process-local arm backend factory (YAML `arm.backend`).
 */

#ifndef AUTOMANIP_ARM_BACKEND_REGISTRY_HPP_
#define AUTOMANIP_ARM_BACKEND_REGISTRY_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "automanip/arm/arm_driver.hpp"
#include "automanip/arm/plant.hpp"

namespace automanip {
namespace arm {

using ArmDriverFactory = ArmDriver* (*)(const std::string& id,
                                        const ArmPlantOptions& options);

/**
 * @brief Maps a backend name to an owning ArmDriver factory.
 *
 * An empty name is rejected. Vendors register with REGISTER_ARM_BACKEND.
 */
class ArmBackendRegistry {
 public:
  static ArmBackendRegistry& Instance();

  void Register(const std::string& name, ArmDriverFactory factory);
  void RegisterAlias(const std::string& alias, const std::string& canonical);

  std::shared_ptr<ArmDriver> Create(const std::string& backend,
                                    const std::string& id,
                                    const ArmPlantOptions& options) const;

 private:
  ArmBackendRegistry() = default;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ArmDriverFactory> factories_;
  std::unordered_map<std::string, std::string> aliases_;
};

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_BACKEND_REGISTRY_HPP_
