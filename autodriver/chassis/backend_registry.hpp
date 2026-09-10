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
 * @brief Process-local chassis backend factory registry.
 */

#ifndef AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "chassis/chassis_driver.hpp"
#include "chassis/types.hpp"

namespace autodriver {
namespace chassis {

/**
 * @struct autodriver::chassis::ChassisBackendPolicy
 * @brief BackendRegistry Policy for chassis (empty backend → "stub").
 */
struct ChassisBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "stub";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix = "unknown chassis backend: ";
};

/**
 * @brief Maps YAML `chassis.backend` → ChassisDriver factory.
 *
 * Alias of BackendRegistry with ChassisBackendPolicy.
 * Internally uses NamedProductFactory; built-in: stub (alias sim).
 */
using ChassisBackendRegistry =
    BackendRegistry<ChassisDriver, ChassisId, ChassisBackendPolicy>;

/**
 * @brief Creator for ChassisBackendRegistry: returns owning ChassisDriver*.
 */
using ChassisDriverFactory = ChassisBackendRegistry::DriverFactory;

/**
 * @brief Register a canonical chassis backend plus optional aliases.
 * @param name Canonical backend string (e.g. "stub").
 * @param factory ChassisDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterChassisBackendWithAliases(
    const std::string& name, ChassisDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    ChassisBackendRegistry::RegisterWithAliases(name, std::move(factory),
                                                aliases);
}

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_BACKEND_REGISTRY_HPP_
