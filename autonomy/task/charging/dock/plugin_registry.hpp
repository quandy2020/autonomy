/*
 * Copyright 2026 The Openbot Authors
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
 * @file plugin_registry.hpp
 * @brief REGISTER_CHARGING_DOCK factory (replaces pluginlib).
 */

#pragma once

#include <string>

#include "autonomy/common/factory.hpp"
#include "autonomy/task/charging/dock/charging_dock.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @brief Process-wide ChargingDock factory keyed by plugin type string.
 * @return Singleton factory.
 */
inline common::Factory<std::string, ChargingDock>& DockPlugins() {
    static common::Factory<std::string, ChargingDock> factory;
    return factory;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#define REGISTER_CHARGING_DOCK(Class, id)                                      \
    namespace {                                                                \
    const bool kRegistered##Class = []() {                                     \
        ::autonomy::task::charging::DockPlugins().Register(                    \
            id, []() -> ::autonomy::task::charging::ChargingDock* {            \
                return new Class();                                            \
            });                                                                \
        return true;                                                           \
    }();                                                                       \
    }
