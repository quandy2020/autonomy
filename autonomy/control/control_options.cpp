/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/control/control_options.hpp"

#include <functional>

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/map/map_options.hpp"

namespace autonomy {
namespace control {
namespace {

void LoadStringArray(
    autonomy::common::LuaParameterDictionary* dict,
    const std::function<void(const std::string&)>& add_fn) {
    const auto values = dict->GetArrayValuesAsStrings();
    for (const auto& value : values) {
        add_fn(value);
    }
}

}  // namespace

proto::ControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::ControllerOptions options;
    if (!parameter_dictionary) {
        return options;
    }

    if (parameter_dictionary->HasKey("controller_frequency")) {
        options.set_controller_frequency(
            parameter_dictionary->GetDouble("controller_frequency"));
    }
    if (parameter_dictionary->HasKey("failure_tolerance")) {
        options.set_failure_tolerance(
            parameter_dictionary->GetDouble("failure_tolerance"));
    }
    if (parameter_dictionary->HasKey("publish_zero_velocity")) {
        options.set_publish_zero_velocity(
            parameter_dictionary->GetBool("publish_zero_velocity"));
    }
    if (parameter_dictionary->HasKey("costmap_2d_options")) {
        *options.mutable_costmap_2d_options() = map::CreateCostmap2DOptions(
            parameter_dictionary->GetDictionary("costmap_2d_options").get());
    }
    if (parameter_dictionary->HasKey("controller_plugins")) {
        LoadStringArray(
            parameter_dictionary->GetDictionary("controller_plugins").get(),
            [&options](const std::string& v) {
                options.add_controller_plugins(v);
            });
    }
    if (parameter_dictionary->HasKey("controller_plugin_libraries")) {
        LoadStringArray(
            parameter_dictionary->GetDictionary("controller_plugin_libraries")
                .get(),
            [&options](const std::string& v) {
                options.add_controller_plugin_libraries(v);
            });
    }

    return options;
}

}  // namespace control
}  // namespace autonomy
