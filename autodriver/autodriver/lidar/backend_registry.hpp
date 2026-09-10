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
 * @brief Process-local lidar_3d backend factory registry.
 */

#ifndef AUTODRIVER_LIDAR_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_LIDAR_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace lidar {

/**
 * @struct autodriver::lidar::LidarBackendPolicy
 * @brief BackendRegistry Policy for 3D lidar (empty backend → "velodyne").
 */
struct LidarBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "velodyne";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix = "unsupported lidar3d backend: ";
};

/**
 * @brief Maps YAML `lidar_3d.backend` → SensorDriver factory.
 *
 * Alias of BackendRegistry with LidarBackendPolicy.
 * Built-ins: velodyne (alias udp), hesai (alias pandar), livox; stubs for
 * rslidar / lslidar / seyond / vanjee.
 */
using LidarBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, LidarBackendPolicy>;

/**
 * @brief Creator for LidarBackendRegistry: returns owning SensorDriver*.
 */
using LidarDriverFactory = LidarBackendRegistry::DriverFactory;

/**
 * @brief Register a canonical 3D lidar backend plus optional aliases.
 * @param name Canonical backend string (e.g. "velodyne").
 * @param factory LidarDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterLidarBackendWithAliases(
    const std::string& name, LidarDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    LidarBackendRegistry::RegisterWithAliases(name, std::move(factory), aliases);
}

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_BACKEND_REGISTRY_HPP_
