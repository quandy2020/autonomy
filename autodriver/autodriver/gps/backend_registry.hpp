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
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace gps {

/**
 * @struct autodriver::gps::GpsBackendPolicy
 * @brief BackendRegistry Policy for GPS (empty backend → "serial").
 */
struct GpsBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "serial";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix = "unknown gps backend: ";
};

/**
 * @brief Maps YAML `gps.backend` → SensorDriver factory.
 *
 * Alias of BackendRegistry with GpsBackendPolicy.
 * Built-ins: serial (NMEA), can. Streaming parsers live in GnssParserRegistry.
 */
using GpsBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, GpsBackendPolicy>;

/**
 * @brief Creator for GpsBackendRegistry: returns owning SensorDriver*.
 */
using GpsDriverFactory = GpsBackendRegistry::DriverFactory;

/**
 * @brief Register a canonical GPS backend plus optional aliases (static init).
 * @param name Canonical backend string (e.g. "serial", "can").
 * @param factory GpsDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterGpsBackendWithAliases(
    const std::string& name, GpsDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    GpsBackendRegistry::RegisterWithAliases(name, std::move(factory), aliases);
}

}  // namespace gps
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_BACKEND_REGISTRY_HPP_
