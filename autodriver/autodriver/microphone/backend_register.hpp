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
 * @file backend_register.hpp
 * @brief Static initializer macro for microphone backends.
 */

#ifndef AUTODRIVER_MICROPHONE_BACKEND_REGISTER_HPP_
#define AUTODRIVER_MICROPHONE_BACKEND_REGISTER_HPP_

#include "autodriver/microphone/backend_registry.hpp"

/**
 * @brief Register a microphone backend at static init time.
 * @param[in] tag Unique C++ identifier suffix for the registrar object.
 * @param[in] name Canonical backend string (e.g. "respeaker").
 * @param[in] factory Creator returning owning SensorDriver* (nullptr ok for stubs).
 * @param[in] ... Optional alias string literals.
 */
#define REGISTER_MICROPHONE_BACKEND(tag, name, factory, ...)                  \
    namespace {                                                                \
    struct MicrophoneBackendRegistrar_##tag {                                 \
        MicrophoneBackendRegistrar_##tag() {                                  \
            ::autodriver::microphone::RegisterMicrophoneBackendWithAliases(   \
                name, factory, {__VA_ARGS__});                                \
        }                                                                     \
    };                                                                        \
    static MicrophoneBackendRegistrar_##tag                                   \
        g_microphone_backend_registrar_##tag;                                 \
    }

#endif  // AUTODRIVER_MICROPHONE_BACKEND_REGISTER_HPP_
