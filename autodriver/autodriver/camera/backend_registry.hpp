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
 * @brief Process-local camera / point_cloud backend factory registries.
 */

#ifndef AUTODRIVER_CAMERA_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_CAMERA_BACKEND_REGISTRY_HPP_

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
namespace camera {

/** Factory: (SensorId, DriverParams) → SensorDriver (or nullptr). */
using CameraDriverFactory = std::function<std::shared_ptr<SensorDriver>(
    const SensorId& id, const hardware::DriverParams& params)>;

using PointCloudDriverFactory = CameraDriverFactory;

/**
 * @class autodriver::camera::CameraBackendRegistry
 * @brief Maps camera backend name → Image driver factory.
 *
 * YAML `camera` + `backend` resolves here via CameraModule
 * (e.g. realsense, orbbec, smartereye).
 */
class CameraBackendRegistry {
public:
    /** Process-wide singleton. */
    static CameraBackendRegistry& Instance();

    /** Register or replace a factory under @p name. */
    void Register(const std::string& name, CameraDriverFactory factory);

    /** Map @p alias to an already-registered canonical @p name. */
    void RegisterAlias(const std::string& alias, const std::string& canonical);

    /**
     * @brief Construct a driver for @p backend after alias resolve.
     * @return nullptr when unknown or Create returns null.
     */
    std::shared_ptr<SensorDriver> Create(
        const std::string& backend, const SensorId& id,
        const hardware::DriverParams& params) const;

    /** Whether @p backend (or its alias target) is registered. */
    bool Has(const std::string& backend) const;

private:
    CameraBackendRegistry() = default;

    /** Follow aliases until a factory name or unchanged string. */
    std::string Resolve(const std::string& backend) const;

    mutable std::mutex mutex_;
    // backend name → factory.
    std::unordered_map<std::string, CameraDriverFactory> factories_;
    // alias → canonical name.
    std::unordered_map<std::string, std::string> aliases_;
};

/**
 * @class autodriver::camera::PointCloudBackendRegistry
 * @brief Maps point_cloud backend name → PointCloud2 driver factory.
 *
 * YAML `point_cloud` + `backend` resolves here via PointCloudModule.
 */
class PointCloudBackendRegistry {
public:
    /** Process-wide singleton. */
    static PointCloudBackendRegistry& Instance();

    /** Register or replace a factory under @p name. */
    void Register(const std::string& name, PointCloudDriverFactory factory);

    /** Map @p alias to an already-registered canonical @p name. */
    void RegisterAlias(const std::string& alias, const std::string& canonical);

    /**
     * @brief Construct a driver for @p backend after alias resolve.
     * @return nullptr when unknown or Create returns null.
     */
    std::shared_ptr<SensorDriver> Create(
        const std::string& backend, const SensorId& id,
        const hardware::DriverParams& params) const;

    /** Whether @p backend (or its alias target) is registered. */
    bool Has(const std::string& backend) const;

private:
    PointCloudBackendRegistry() = default;

    /** Follow aliases until a factory name or unchanged string. */
    std::string Resolve(const std::string& backend) const;

    mutable std::mutex mutex_;
    // backend name → factory.
    std::unordered_map<std::string, PointCloudDriverFactory> factories_;
    // alias → canonical name.
    std::unordered_map<std::string, std::string> aliases_;
};

/**
 * @brief Register @p name and optional @p aliases in one call (used by macros).
 */
void RegisterCameraBackendWithAliases(
    const std::string& name, CameraDriverFactory factory,
    std::initializer_list<const char*> aliases);

/**
 * @brief Register @p name and optional @p aliases in one call (used by macros).
 */
void RegisterPointCloudBackendWithAliases(
    const std::string& name, PointCloudDriverFactory factory,
    std::initializer_list<const char*> aliases);

}  // namespace camera
}  // namespace autodriver

#endif  // AUTODRIVER_CAMERA_BACKEND_REGISTRY_HPP_
