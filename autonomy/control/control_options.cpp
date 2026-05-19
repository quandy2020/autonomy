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

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/controller/graceful_controller/parameter_options.hpp"
#include "autonomy/map/map_options.hpp"

namespace autonomy {
namespace control {

proto::ControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::ControllerOptions options;

    if (!parameter_dictionary) {
        return options;
    }

    if (parameter_dictionary->HasKey("costmap_2d")) {
        auto costmap_dict = parameter_dictionary->GetDictionary("costmap_2d");
        if (costmap_dict && costmap_dict->HasKey("costmap_2d")) {
            auto nested_costmap_dict =
                costmap_dict->GetDictionary("costmap_2d");
            if (nested_costmap_dict) {
                *options.mutable_costmap_2d_options() =
                    map::CreateCostmap2DOptions(nested_costmap_dict.get());
            }
        }
    }

    if (parameter_dictionary->HasKey("graceful_controller")) {
        auto graceful_dict =
            parameter_dictionary->GetDictionary("graceful_controller");
        if (graceful_dict) {
            *options.mutable_graceful_controller_options() =
                controller::graceful_controller::LoadOptions(
                    graceful_dict.get());
        }
    }
    return options;
}

}  // namespace control
}  // namespace autonomy
