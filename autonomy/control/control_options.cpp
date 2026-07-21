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
#include "autonomy/control/controller/graceful_controller/parameter_options.hpp"
#include "autonomy/control/controller/mppi_controller/tools/mppi_options.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/parameter_options.hpp"
#include "autonomy/control/controller/teb_controller/tools/teb_options.hpp"
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
    if (parameter_dictionary->HasKey("costmap")) {
        *options.mutable_costmap_2d_options() = map::CreateCostmap2DOptions(
            parameter_dictionary->GetDictionary("costmap").get());
    } else if (parameter_dictionary->HasKey("costmap_2d_options")) {
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

    auto* checker_opts = options.mutable_checker_options();
    if (parameter_dictionary->HasKey("goal_checker")) {
        auto dict = parameter_dictionary->GetDictionary("goal_checker");
        if (dict) {
            auto* gc = checker_opts->mutable_goal_checker();
            if (dict->HasKey("plugin")) {
                gc->set_plugin(dict->GetString("plugin"));
            }
            if (dict->HasKey("stateful")) {
                gc->set_stateful(dict->GetBool("stateful"));
            }
            if (dict->HasKey("xy_goal_tolerance")) {
                gc->set_xy_goal_tolerance(
                    dict->GetDouble("xy_goal_tolerance"));
            }
            if (dict->HasKey("yaw_goal_tolerance")) {
                gc->set_yaw_goal_tolerance(
                    dict->GetDouble("yaw_goal_tolerance"));
            }
            if (dict->HasKey("path_length_tolerance")) {
                gc->set_path_length_tolerance(
                    dict->GetDouble("path_length_tolerance"));
            }
        }
    }
    if (parameter_dictionary->HasKey("progress_checker")) {
        auto dict = parameter_dictionary->GetDictionary("progress_checker");
        if (dict) {
            auto* pc = checker_opts->mutable_progress_checker();
            if (dict->HasKey("plugin")) {
                pc->set_plugin(dict->GetString("plugin"));
            }
            if (dict->HasKey("required_movement_radius")) {
                pc->set_required_movement_radius(
                    dict->GetDouble("required_movement_radius"));
            }
            if (dict->HasKey("movement_time_allowance")) {
                pc->set_movement_time_allowance(
                    dict->GetDouble("movement_time_allowance"));
            }
        }
    }

    if (parameter_dictionary->HasKey("mppi_controller")) {
        *options.mutable_mppi_controller_options() =
            controller::mppi_controller::tools::LoadOptions(
                parameter_dictionary
                    ->GetNonReferenceCountedDictionary("mppi_controller")
                    .get());
    }
    if (parameter_dictionary->HasKey("graceful_controller")) {
        *options.mutable_graceful_controller_options() =
            controller::graceful_controller::LoadOptions(
                parameter_dictionary
                    ->GetNonReferenceCountedDictionary("graceful_controller")
                    .get());
    }
    if (parameter_dictionary->HasKey("pure_pursuit_controller")) {
        *options.mutable_pure_pursuit_controller_options() =
            controller::pure_pursuit_controller::LoadOptions(
                parameter_dictionary
                    ->GetNonReferenceCountedDictionary(
                        "pure_pursuit_controller")
                    .get());
    }
    if (parameter_dictionary->HasKey("teb_controller")) {
        *options.mutable_teb_controller_options() =
            controller::teb_controller::tools::LoadOptions(
                parameter_dictionary
                    ->GetNonReferenceCountedDictionary("teb_controller")
                    .get());
    }

    return options;
}

}  // namespace control
}  // namespace autonomy
