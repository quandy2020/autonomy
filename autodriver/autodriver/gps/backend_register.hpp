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
 * @brief Static initializer macro for GPS backends.
 */

#ifndef AUTODRIVER_GPS_BACKEND_REGISTER_HPP_
#define AUTODRIVER_GPS_BACKEND_REGISTER_HPP_

#include "autodriver/gps/backend_registry.hpp"

/**
 * @brief Register a GPS backend at static init time.
 * @param tag Unique C++ identifier suffix for the registrar object.
 * @param name Canonical backend string (e.g. "serial").
 * @param factory Creator returning owning SensorDriver* (nullptr ok for stubs).
 * @param ... Optional alias string literals.
 */
#define REGISTER_GPS_BACKEND(tag, name, factory, ...)                         \
    namespace {                                                                \
    struct GpsBackendRegistrar_##tag {                                        \
        GpsBackendRegistrar_##tag() {                                         \
            ::autodriver::gps::RegisterGpsBackendWithAliases(                 \
                name, factory, {__VA_ARGS__});                                \
        }                                                                     \
    };                                                                        \
    static GpsBackendRegistrar_##tag g_gps_backend_registrar_##tag;           \
    }

#endif  // AUTODRIVER_GPS_BACKEND_REGISTER_HPP_
