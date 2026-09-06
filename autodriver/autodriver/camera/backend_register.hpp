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
 * @brief Static registration macros for camera / point_cloud backends.
 */

#ifndef AUTODRIVER_CAMERA_BACKEND_REGISTER_HPP_
#define AUTODRIVER_CAMERA_BACKEND_REGISTER_HPP_

#include "autodriver/camera/backend_registry.hpp"

/**
 * @brief Register a camera backend at static init time.
 * @param tag Unique C++ identifier suffix for the registrar object.
 * @param name Canonical backend string (e.g. "realsense").
 * @param factory Create function (may return nullptr without SDK).
 * @param ... Optional alias string literals.
 */
#define REGISTER_CAMERA_BACKEND(tag, name, factory, ...)                      \
    namespace {                                                                \
    struct CameraBackendRegistrar_##tag {                                     \
        CameraBackendRegistrar_##tag() {                                      \
            ::autodriver::camera::RegisterCameraBackendWithAliases(           \
                name, factory, {__VA_ARGS__});                                \
        }                                                                     \
    };                                                                        \
    static CameraBackendRegistrar_##tag g_camera_backend_registrar_##tag;     \
    }

/**
 * @brief Register a point_cloud backend at static init time.
 * @param tag Unique C++ identifier suffix for the registrar object.
 * @param name Canonical backend string (e.g. "realsense").
 * @param factory Create function (may return nullptr without SDK).
 * @param ... Optional alias string literals.
 */
#define REGISTER_POINTCLOUD_BACKEND(tag, name, factory, ...)                  \
    namespace {                                                                \
    struct PointCloudBackendRegistrar_##tag {                                 \
        PointCloudBackendRegistrar_##tag() {                                  \
            ::autodriver::camera::RegisterPointCloudBackendWithAliases(       \
                name, factory, {__VA_ARGS__});                                \
        }                                                                     \
    };                                                                        \
    static PointCloudBackendRegistrar_##tag                                   \
        g_pointcloud_backend_registrar_##tag;                                \
    }

#endif  // AUTODRIVER_CAMERA_BACKEND_REGISTER_HPP_
