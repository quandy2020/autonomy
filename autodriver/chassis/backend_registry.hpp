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

/**
 * @file
 * @brief Process-local chassis backend factory registry.
 */

#ifndef AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_

#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "chassis/chassis_driver.hpp"
#include "chassis/types.hpp"
#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace chassis {

using ChassisDriverFactory = std::function<std::shared_ptr<ChassisDriver>(
    const ChassisId& id, const hardware::DriverParams& params)>;

/**
 * @brief Maps YAML `chassis.backend` → ChassisDriver factory.
 */
class ChassisBackendRegistry {
public:
  static ChassisBackendRegistry& Instance();

  void Register(const std::string& name, ChassisDriverFactory factory);
  void RegisterAlias(const std::string& alias, const std::string& canonical);

  std::shared_ptr<ChassisDriver> Create(
      const std::string& backend, const ChassisId& id,
      const hardware::DriverParams& params) const;

  bool Has(const std::string& backend) const;

private:
  ChassisBackendRegistry() = default;
  std::string Resolve(const std::string& backend) const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ChassisDriverFactory> factories_;
  std::unordered_map<std::string, std::string> aliases_;
};

void RegisterChassisBackendWithAliases(
    const std::string& name, ChassisDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_
