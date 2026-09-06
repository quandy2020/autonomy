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
 * @brief Static initializer macro for radar backends.
 */

#ifndef AUTODRIVER_RADAR_BACKEND_REGISTER_HPP_
#define AUTODRIVER_RADAR_BACKEND_REGISTER_HPP_

#include "autodriver/radar/backend_registry.hpp"

/**
 * @brief Register a radar backend at static init time.
 * @param tag Unique C++ identifier suffix for the registrar object.
 * @param name Canonical backend string (e.g. "conti").
 * @param factory Create function (may return nullptr for stubs).
 * @param ... Optional alias string literals (e.g. "continental").
 */
#define REGISTER_RADAR_BACKEND(tag, name, factory, ...)                       \
    namespace {                                                                \
    struct RadarBackendRegistrar_##tag {                                      \
        RadarBackendRegistrar_##tag() {                                       \
            ::autodriver::radar::RegisterRadarBackendWithAliases(             \
                name, factory, {__VA_ARGS__});                                \
        }                                                                     \
    };                                                                        \
    static RadarBackendRegistrar_##tag g_radar_backend_registrar_##tag;       \
    }

#endif  // AUTODRIVER_RADAR_BACKEND_REGISTER_HPP_
