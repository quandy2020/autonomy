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
 * @brief Static registration macro for chassis backends.
 */

#ifndef AUTODRIVER_CHASSIS_BACKEND_REGISTER_HPP_
#define AUTODRIVER_CHASSIS_BACKEND_REGISTER_HPP_

#include "chassis/backend_registry.hpp"

/**
 * @brief Register a chassis backend factory at static initialization time.
 *
 * Expands to a file-local registrar that calls
 * RegisterChassisBackendWithAliases once before main().
 * @param tag Unique C++ identifier suffix for the registrar type / object.
 * @param name Canonical backend string (e.g. "stub", "scout").
 * @param factory Creator returning owning ChassisDriver* (e.g. CreateStubChassisDriver).
 * @param ... Optional alias string literals (e.g. "sim", "fake").
 */
#define REGISTER_CHASSIS_BACKEND(tag, name, factory, ...)                      \
  namespace {                                                                  \
  struct ChassisBackendRegistrar_##tag {                                       \
    ChassisBackendRegistrar_##tag() {                                          \
      ::autodriver::chassis::RegisterChassisBackendWithAliases(                \
          name, factory, {__VA_ARGS__});                                       \
    }                                                                          \
  };                                                                           \
  static ChassisBackendRegistrar_##tag g_chassis_backend_registrar_##tag;      \
  }

#endif  // AUTODRIVER_CHASSIS_BACKEND_REGISTER_HPP_
