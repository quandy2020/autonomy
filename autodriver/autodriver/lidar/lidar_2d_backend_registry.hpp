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
 * @brief Process-local lidar_2d backend factory registry.
 */

#ifndef AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace lidar {

/**
 * @struct autodriver::lidar::Lidar2dBackendPolicy
 * @brief BackendRegistry Policy for 2D lidar (empty backend → "rplidar").
 */
struct Lidar2dBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "rplidar";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix =
        "unsupported lidar2d backend: ";
};

/**
 * @brief Maps YAML `lidar_2d.backend` → SensorDriver factory.
 *
 * Alias of BackendRegistry with Lidar2dBackendPolicy.
 * Built-ins: rplidar (alias slamtec).
 */
using Lidar2dBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, Lidar2dBackendPolicy>;

/**
 * @brief Creator for Lidar2dBackendRegistry: returns owning SensorDriver*.
 */
using Lidar2dDriverFactory = Lidar2dBackendRegistry::DriverFactory;

/**
 * @brief Register a canonical 2D lidar backend plus optional aliases.
 * @param name Canonical backend string (e.g. "rplidar").
 * @param factory Lidar2dDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterLidar2dBackendWithAliases(
    const std::string& name, Lidar2dDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    Lidar2dBackendRegistry::RegisterWithAliases(name, std::move(factory),
                                                aliases);
}

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTRY_HPP_
