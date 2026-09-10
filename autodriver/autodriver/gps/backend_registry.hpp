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
 * @brief Process-local GPS backend factory registry.
 */

#ifndef AUTODRIVER_GPS_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_GPS_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>

#include "autodriver/common/named_factory.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace gps {

/**
 * @brief Creator for autolink::common::Factory: returns owning SensorDriver*.
 * @param id Sensor instance id from YAML (e.g. "gps/main").
 * @param params Backend-specific key/value map from YAML.
 * @return New SensorDriver, or nullptr when construction fails.
 */
using GpsDriverFactory =
    NamedProductFactory<SensorDriver, SensorId>::Creator;

/**
 * @class autodriver::gps::GpsBackendRegistry
 * @brief Maps YAML `gps.backend` → SensorDriver factory.
 *
 * Internally uses NamedProductFactory (autolink::common::Factory).
 * Built-ins: serial (NMEA), can. Streaming parsers live in GnssParserRegistry.
 */
class GpsBackendRegistry {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(GpsBackendRegistry)

  /**
   * @brief Access the process-wide singleton (static-init backends register here).
   * @return Reference to the unique GpsBackendRegistry instance.
   */
  static GpsBackendRegistry& Instance();

  /**
   * @brief Register or replace a factory under a canonical backend name.
   * @param name Canonical backend string (e.g. "serial", "can").
   * @param factory Creator returning new SensorDriver*.
   */
  void RegisterBackend(const std::string& name, GpsDriverFactory factory);

  /**
   * @brief Map an alias onto an already-registered canonical backend name.
   * @param alias Alternate YAML name.
   * @param canonical Existing registered name.
   */
  void RegisterBackendAlias(const std::string& alias,
                            const std::string& canonical);

  /**
   * @brief Create a SensorDriver for @p backend (empty string → "serial").
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
   * @param backend Canonical name or alias (empty → "serial").
   * @return true if a factory is available after alias resolve.
   */
  bool HasBackend(const std::string& backend) const;

private:
  /**
   * @brief Private default constructor for the process-wide singleton.
   */
  GpsBackendRegistry() = default;

  NamedProductFactory<SensorDriver, SensorId> factory_;
};

/**
 * @brief Register a canonical backend plus optional aliases in one call.
 *
 * Used by REGISTER_GPS_BACKEND at static init.
 * @param name Canonical backend string.
 * @param factory GpsDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
void RegisterGpsBackendWithAliases(
    const std::string& name, GpsDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace gps
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_BACKEND_REGISTRY_HPP_
