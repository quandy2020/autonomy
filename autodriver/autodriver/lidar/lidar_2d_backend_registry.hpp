/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Process-local lidar_2d backend factory registry.
 */

#ifndef AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTRY_HPP_

#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace lidar {

using Lidar2dDriverFactory = std::function<std::shared_ptr<SensorDriver>(
    const SensorId& id, const hardware::DriverParams& params)>;

/**
 * @class autodriver::lidar::Lidar2dBackendRegistry
 * @brief Maps lidar_2d backend name → driver factory (e.g. rplidar / slamtec).
 */
class Lidar2dBackendRegistry {
public:
    static Lidar2dBackendRegistry& Instance();

    void Register(const std::string& name, Lidar2dDriverFactory factory);
    void RegisterAlias(const std::string& alias, const std::string& canonical);

    std::shared_ptr<SensorDriver> Create(
        const std::string& backend, const SensorId& id,
        const hardware::DriverParams& params) const;

    bool Has(const std::string& backend) const;

private:
    Lidar2dBackendRegistry() = default;
    std::string Resolve(const std::string& backend) const;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, Lidar2dDriverFactory> factories_;
    std::unordered_map<std::string, std::string> aliases_;
};

void RegisterLidar2dBackendWithAliases(
    const std::string& name, Lidar2dDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTRY_HPP_
