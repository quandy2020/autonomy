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
#include "autonomy/control/controller/nmpc_controller/parameter_options.hpp"
#include "autonomy/control/controller/tdmpc_controller/parameter_options.hpp"
#include "autonomy/map/map_options.hpp"

namespace autonomy {
namespace control {
namespace {

void LoadGoalCheckerFromDict(
    ::autonomy::common::LuaParameterDictionary* dict,
    proto::GoalCheckerOptions* goal) {
    if (!dict || !goal) {
        return;
    }
    if (dict->HasKey("xy_goal_tolerance")) {
        goal->set_xy_goal_tolerance(dict->GetDouble("xy_goal_tolerance"));
    }
    if (dict->HasKey("yaw_goal_tolerance")) {
        goal->set_yaw_goal_tolerance(dict->GetDouble("yaw_goal_tolerance"));
    }
    if (dict->HasKey("stateful")) {
        goal->set_stateful(dict->GetBool("stateful"));
    }
}

void LoadProgressCheckerFromDict(
    ::autonomy::common::LuaParameterDictionary* dict,
    proto::ProgressCheckerOptions* progress) {
    if (!dict || !progress) {
        return;
    }
    if (dict->HasKey("required_movement_radius")) {
        progress->set_required_movement_radius(
            dict->GetDouble("required_movement_radius"));
    }
    if (dict->HasKey("movement_time_allowance")) {
        progress->set_movement_time_allowance(
            dict->GetDouble("movement_time_allowance"));
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
        auto costmap_dict =
            parameter_dictionary->GetNonReferenceCountedDictionary("costmap");
        *options.mutable_costmap_2d_options() =
            map::CreateCostmap2DOptions(costmap_dict.get());
    }

    if (parameter_dictionary->HasKey("goal_checker")) {
        auto goal_dict = parameter_dictionary->GetDictionary("goal_checker");
        LoadGoalCheckerFromDict(
            goal_dict.get(),
            options.mutable_checker_options()->mutable_goal_checker());
    }
    if (parameter_dictionary->HasKey("progress_checker")) {
        auto progress_dict =
            parameter_dictionary->GetDictionary("progress_checker");
        LoadProgressCheckerFromDict(
            progress_dict.get(),
            options.mutable_checker_options()->mutable_progress_checker());
    }

    if (parameter_dictionary->HasKey("mppi_controller")) {
        auto mppi_dict = parameter_dictionary->GetNonReferenceCountedDictionary(
            "mppi_controller");
        if (mppi_dict->HasKey("goal_checker")) {
            auto goal_dict = mppi_dict->GetDictionary("goal_checker");
            LoadGoalCheckerFromDict(
                goal_dict.get(),
                options.mutable_checker_options()->mutable_goal_checker());
        }
        if (mppi_dict->HasKey("progress_checker")) {
            auto progress_dict =
                mppi_dict->GetDictionary("progress_checker");
            LoadProgressCheckerFromDict(
                progress_dict.get(),
                options.mutable_checker_options()->mutable_progress_checker());
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

    if (parameter_dictionary->HasKey("nmpc_controller")) {
        auto nmpc_dict =
            parameter_dictionary->GetDictionary("nmpc_controller");
        if (nmpc_dict) {
            *options.mutable_nmpc_controller_options() =
                controller::nmpc::LoadOptions(nmpc_dict.get());
        }
    }

    if (parameter_dictionary->HasKey("tdmpc_controller")) {
        auto tdmpc_dict =
            parameter_dictionary->GetDictionary("tdmpc_controller");
        if (tdmpc_dict) {
            *options.mutable_tdmpc_controller_options() =
                controller::tdmpc::LoadOptions(tdmpc_dict.get());
        }
    }
    return options;
}

}  // namespace control
}  // namespace autonomy
