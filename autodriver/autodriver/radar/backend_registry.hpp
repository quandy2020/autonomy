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
 * @brief Process-local radar backend factory registry.
 */

#ifndef AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>

#include "autodriver/common/named_factory.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace radar {

/**
 * @brief Creator for autolink::common::Factory: returns owning SensorDriver*.
 * @param id Sensor instance id from YAML (e.g. "radar/front").
 * @param params Backend-specific key/value map from YAML.
 * @return New SensorDriver, or nullptr when construction fails / stub.
 */
using RadarDriverFactory =
    NamedProductFactory<SensorDriver, SensorId>::Creator;

/**
 * @class autodriver::radar::RadarBackendRegistry
 * @brief Maps YAML `radar.backend` → SensorDriver factory.
 *
 * Internally uses NamedProductFactory (autolink::common::Factory).
 * Default backend when empty: "conti" (may be stub → nullptr).
 */
class RadarBackendRegistry {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(RadarBackendRegistry)

  /**
   * @brief Access the process-wide singleton (static-init backends register here).
   * @return Reference to the unique RadarBackendRegistry instance.
   */
  static RadarBackendRegistry& Instance();

  /**
   * @brief Register or replace a factory under a canonical backend name.
   * @param name Canonical backend string (e.g. "conti").
   * @param factory Creator returning new SensorDriver*.
   */
  void RegisterBackend(const std::string& name, RadarDriverFactory factory);

  /**
   * @brief Map an alias onto an already-registered canonical backend name.
   * @param alias Alternate YAML name (e.g. "continental").
   * @param canonical Existing registered name (e.g. "conti").
   */
  void RegisterBackendAlias(const std::string& alias,
                            const std::string& canonical);

  /**
   * @brief Create a SensorDriver for @p backend (empty string → "conti").
   * @param backend YAML `backend` or alias.
   * @param id Sensor instance id passed to the factory.
   * @param params YAML params (and shorthand merges).
   * @return Shared driver, or nullptr if unknown / creator returns null.
   */
  SensorDriver::SharedPtr CreateDriver(
      const std::string& backend, const SensorId& id,
      const hardware::DriverParams& params) const;

  /**
   * @brief Check whether @p backend resolves to a registered factory.
   * @param backend Canonical name or alias (empty → "conti").
   * @return true if a factory is available after alias resolve.
   */
  bool HasBackend(const std::string& backend) const;

private:
  /**
   * @brief Private default constructor for the process-wide singleton.
   */
  RadarBackendRegistry() = default;

  NamedProductFactory<SensorDriver, SensorId> factory_;
};

/**
 * @brief Register a canonical backend plus optional aliases in one call.
 *
 * Used by REGISTER_RADAR_BACKEND at static init.
 * @param name Canonical backend string.
 * @param factory RadarDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
void RegisterRadarBackendWithAliases(
    const std::string& name, RadarDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace radar
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_
