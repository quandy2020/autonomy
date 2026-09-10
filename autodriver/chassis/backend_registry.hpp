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
#include <string>

#include "chassis/chassis_driver.hpp"
#include "chassis/types.hpp"
#include "autodriver/common/named_factory.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief Creator for autolink::common::Factory: returns owning ChassisDriver*.
 * @param id Chassis instance id from YAML (e.g. "chassis/base").
 * @param params Backend-specific key/value map from YAML `chassis.params`.
 * @return New ChassisDriver, or nullptr when construction fails.
 */
using ChassisDriverFactory = NamedProductFactory<ChassisDriver, ChassisId>::Creator;

/**
 * @brief Maps YAML `chassis.backend` → ChassisDriver factory.
 *
 * Internally uses autodriver::NamedProductFactory (autolink::common::Factory).
 */
class ChassisBackendRegistry {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(ChassisBackendRegistry)

  /**
   * @brief Access the process-wide singleton (static-init backends register here).
   * @return Reference to the unique ChassisBackendRegistry instance.
   */
  static ChassisBackendRegistry& Instance();

  /**
   * @brief Register or replace a factory under a canonical backend name.
   * @param name Canonical backend string (e.g. "stub", "scout").
   * @param factory Creator returning new ChassisDriver*.
   */
  void RegisterBackend(const std::string& name, ChassisDriverFactory factory);

  /**
   * @brief Map an alias onto an already-registered canonical backend name.
   * @param alias Alternate YAML name (e.g. "sim").
   * @param canonical Existing registered name (e.g. "stub").
   */
  void RegisterBackendAlias(const std::string& alias,
                            const std::string& canonical);

  /**
   * @brief Create a ChassisDriver for @p backend (empty string → "stub").
   * @param backend YAML `chassis.backend` or alias.
   * @param id Chassis instance id passed to the factory.
   * @param params YAML `chassis.params` (and shorthand merges).
   * @return Shared driver, or nullptr if the name / alias is unknown or the
   *         factory returns null.
   */
  ChassisDriver::SharedPtr CreateDriver(
      const std::string& backend, const ChassisId& id,
      const hardware::DriverParams& params) const;

  /**
   * @brief Check whether @p backend resolves to a registered factory.
   * @param backend Canonical name or alias.
   * @return true if a factory is available after alias resolve.
   */
  bool HasBackend(const std::string& backend) const;

private:
  /**
   * @brief Private default constructor for the process-wide singleton.
   */
  ChassisBackendRegistry() = default;

  NamedProductFactory<ChassisDriver, ChassisId> factory_;
};

/**
 * @brief Register a canonical backend plus optional aliases in one call.
 *
 * Used by REGISTER_CHASSIS_BACKEND at static init.
 * @param name Canonical backend string.
 * @param factory ChassisDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty entries skipped).
 */
void RegisterChassisBackendWithAliases(
    const std::string& name, ChassisDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_
