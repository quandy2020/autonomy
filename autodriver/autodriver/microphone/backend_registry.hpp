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
 * @brief Process-local microphone backend factory registry.
 */

#ifndef AUTODRIVER_MICROPHONE_BACKEND_REGISTRY_HPP_
#define AUTODRIVER_MICROPHONE_BACKEND_REGISTRY_HPP_

#include <initializer_list>
#include <string>
#include <utility>

#include "autodriver/common/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace microphone {

/**
 * @struct autodriver::microphone::MicrophoneBackendPolicy
 * @brief BackendRegistry Policy for microphone (empty backend → "respeaker").
 */
struct MicrophoneBackendPolicy {
    /** @brief Default YAML backend when the string is empty. */
    static constexpr const char* kDefaultBackend = "respeaker";
    /** @brief AERROR prefix when CreateDriver fails. */
    static constexpr const char* kUnknownPrefix = "unknown microphone backend: ";
};

/**
 * @brief Maps YAML `microphone.backend` → SensorDriver factory.
 *
 * Alias of BackendRegistry with MicrophoneBackendPolicy.
 * Built-ins: respeaker (stub until SDK wired).
 */
using MicrophoneBackendRegistry =
    BackendRegistry<SensorDriver, SensorId, MicrophoneBackendPolicy>;

/**
 * @brief Creator for MicrophoneBackendRegistry: returns owning SensorDriver*.
 */
using MicrophoneDriverFactory = MicrophoneBackendRegistry::DriverFactory;

/**
 * @brief Register a canonical microphone backend plus optional aliases.
 * @param name Canonical backend string (e.g. "respeaker").
 * @param factory MicrophoneDriverFactory for @p name.
 * @param aliases Optional null-terminated C string aliases (empty skipped).
 */
inline void RegisterMicrophoneBackendWithAliases(
    const std::string& name, MicrophoneDriverFactory factory,
    std::initializer_list<const char*> aliases) {
    MicrophoneBackendRegistry::RegisterWithAliases(name, std::move(factory),
                                                   aliases);
}

}  // namespace microphone
}  // namespace autodriver

#endif  // AUTODRIVER_MICROPHONE_BACKEND_REGISTRY_HPP_
