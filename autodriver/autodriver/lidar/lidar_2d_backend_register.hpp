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
 * @brief Static registration macro for lidar_2d backends.
 */

#ifndef AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTER_HPP_
#define AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTER_HPP_

#include "autodriver/lidar/lidar_2d_backend_registry.hpp"

/**
 * @brief Register a lidar_2d backend at static init time.
 * @param tag Unique C++ identifier suffix for the registrar object.
 * @param name Canonical backend string (e.g. "rplidar").
 * @param factory Creator returning owning SensorDriver* (nullptr ok for stubs).
 * @param ... Optional alias string literals (e.g. "slamtec").
 */
#define REGISTER_LIDAR2D_BACKEND(tag, name, factory, ...)                   \
    namespace {                                                              \
    struct Lidar2dBackendRegistrar_##tag {                                  \
        Lidar2dBackendRegistrar_##tag() {                                   \
            ::autodriver::lidar::RegisterLidar2dBackendWithAliases(         \
                name, factory, {__VA_ARGS__});                              \
        }                                                                   \
    };                                                                      \
    static Lidar2dBackendRegistrar_##tag g_lidar2d_backend_registrar_##tag; \
    }

#endif  // AUTODRIVER_LIDAR_LIDAR_2D_BACKEND_REGISTER_HPP_
