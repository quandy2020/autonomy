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
namespace radar {

/** Factory: (SensorId, DriverParams) → SensorDriver (or nullptr for stubs). */
using RadarDriverFactory = std::function<std::shared_ptr<SensorDriver>(
    const SensorId& id, const hardware::DriverParams& params)>;

/**
 * @class autodriver::radar::RadarBackendRegistry
 * @brief Maps radar backend name → driver factory.
 *
 * YAML `backend: conti` (alias `continental`) resolves here via RadarModule.
 */
class RadarBackendRegistry {
public:
    /** Process-wide singleton. */
    static RadarBackendRegistry& Instance();

    /**
     * @brief Register or replace a factory under @p name.
     */
    void Register(const std::string& name, RadarDriverFactory factory);

    /**
     * @brief Map @p alias to an already-registered canonical @p name.
     */
    void RegisterAlias(const std::string& alias, const std::string& canonical);

    /**
     * @brief Construct a driver for @p backend after alias resolve.
     * @return nullptr when unknown or stub Create returns null.
     */
    std::shared_ptr<SensorDriver> Create(
        const std::string& backend, const SensorId& id,
        const hardware::DriverParams& params) const;

    /** Whether @p backend (or its alias target) is registered. */
    bool Has(const std::string& backend) const;

private:
    RadarBackendRegistry() = default;

    /** Follow aliases until a factory name or unchanged string. */
    std::string Resolve(const std::string& backend) const;

    mutable std::mutex mutex_;
    // backend name → factory.
    std::unordered_map<std::string, RadarDriverFactory> factories_;
    // alias → canonical name.
    std::unordered_map<std::string, std::string> aliases_;
};

/**
 * @brief Register @p name and optional @p aliases in one call (used by macros).
 */
void RegisterRadarBackendWithAliases(
    const std::string& name, RadarDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace radar
}  // namespace autodriver

#endif  // AUTODRIVER_RADAR_BACKEND_REGISTRY_HPP_
