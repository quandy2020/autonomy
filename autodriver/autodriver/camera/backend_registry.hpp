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
 * @brief Process-local camera / point_cloud backend factory registries.
 */

#ifndef AUTODRIVER_CAMERA_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_CAMERA_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace camera {

/**
 * @struct autodriver::camera::CameraBackendPolicy
 * @brief BackendRegistry Policy for camera Image drivers (default "realsense").
 */
struct CameraBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "realsense";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix = "unsupported camera backend: ";
};

/**
 * @struct autodriver::camera::PointCloudBackendPolicy
 * @brief BackendRegistry Policy for depth PointCloud2 drivers (default "realsense").
 */
struct PointCloudBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "realsense";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix =
        "unsupported point cloud backend: ";
};

/**
 * @brief Maps YAML camera `backend` → Image SensorDriver factory.
 *
 * Built-ins: realsense, orbbec, smartereye (stub).
 */
using CameraBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, CameraBackendPolicy>;

/**
 * @brief Maps YAML point_cloud `backend` → PointCloud2 SensorDriver factory.
 *
 * Built-ins: realsense, orbbec (require SDK).
 */
using PointCloudBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, PointCloudBackendPolicy>;

/**
 * @brief Creator for Image drivers: returns owning SensorDriver*.
 */
using CameraDriverFactory = CameraBackendRegistry::DriverFactory;

/**
 * @brief Creator for PointCloud2 drivers (same signature as CameraDriverFactory).
 */
using PointCloudDriverFactory = PointCloudBackendRegistry::DriverFactory;

/**
 * @brief Register a camera backend plus optional aliases (static init).
 * @param[in] name Canonical backend string (e.g. "realsense").
 * @param[in] factory CameraDriverFactory for @p name.
 * @param[in] aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterCameraBackendWithAliases(
    const std::string& name, CameraDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    CameraBackendRegistry::RegisterWithAliases(name, std::move(factory),
                                               aliases);
}

/**
 * @brief Register a point_cloud backend plus optional aliases (static init).
 * @param[in] name Canonical backend string (e.g. "realsense").
 * @param[in] factory PointCloudDriverFactory for @p name.
 * @param[in] aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterPointCloudBackendWithAliases(
    const std::string& name, PointCloudDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    PointCloudBackendRegistry::RegisterWithAliases(name, std::move(factory),
                                                   aliases);
}

}  // namespace camera
}  // namespace autodriver

#endif  // AUTODRIVER_CAMERA_BACKEND_REGISTRY_HPP_
