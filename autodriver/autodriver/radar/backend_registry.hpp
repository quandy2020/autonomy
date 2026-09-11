/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file backend_registry.hpp
 * @brief Process-local radar backend factory registry.
 */

#ifndef AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace radar {

/**
 * @struct autodriver::radar::RadarBackendPolicy
 * @brief BackendRegistry Policy for radar (empty backend → "conti").
 */
struct RadarBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "conti";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix = "unknown radar backend: ";
};

/**
 * @brief Maps YAML `radar.backend` → SensorDriver factory.
 *
 * Alias of BackendRegistry with RadarBackendPolicy.
 * Built-ins: conti (alias continental); Create may return nullptr (stub).
 */
using RadarBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, RadarBackendPolicy>;

/**
 * @brief Creator for RadarBackendRegistry: returns owning SensorDriver*.
 */
using RadarDriverFactory = RadarBackendRegistry::DriverFactory;

/**
 * @brief Register a canonical radar backend plus optional aliases.
 * @param[in] name Canonical backend string (e.g. "conti").
 * @param[in] factory RadarDriverFactory for @p name.
 * @param[in] aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterRadarBackendWithAliases(
    const std::string& name, RadarDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    RadarBackendRegistry::RegisterWithAliases(name, std::move(factory), aliases);
}

}  // namespace radar
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_
