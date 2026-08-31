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

/**
 * @file parameter_options.cpp
 * @brief Lua loader for NMPC controller options
 */

#include "autonomy/control/controller/nmpc_controller/parameter_options.hpp"

#include "autonomy/control/controller/nmpc_controller/parameter_defaults.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

proto::NMPCControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* parameter_dictionary) {
    proto::NMPCControllerOptions options;
    if (!parameter_dictionary) {
        return ApplyDefaults(std::move(options));
    }

    if (parameter_dictionary->HasKey("transform_tolerance")) {
        options.set_transform_tolerance(
            parameter_dictionary->GetDouble("transform_tolerance"));
    }
    if (parameter_dictionary->HasKey("model_dt")) {
        options.set_model_dt(parameter_dictionary->GetDouble("model_dt"));
    }
    if (parameter_dictionary->HasKey("horizon_steps")) {
        options.set_horizon_steps(
            static_cast<int32_t>(parameter_dictionary->GetInt("horizon_steps")));
    }
    if (parameter_dictionary->HasKey("max_solver_iter")) {
        options.set_max_solver_iter(static_cast<int32_t>(
            parameter_dictionary->GetInt("max_solver_iter")));
    }
    if (parameter_dictionary->HasKey("max_linear_vel")) {
        options.set_max_linear_vel(
            parameter_dictionary->GetDouble("max_linear_vel"));
    }
    if (parameter_dictionary->HasKey("min_linear_vel")) {
        options.set_min_linear_vel(
            parameter_dictionary->GetDouble("min_linear_vel"));
    }
    if (parameter_dictionary->HasKey("max_angular_vel")) {
        options.set_max_angular_vel(
            parameter_dictionary->GetDouble("max_angular_vel"));
    }
    if (parameter_dictionary->HasKey("lookahead_dist")) {
        options.set_lookahead_dist(
            parameter_dictionary->GetDouble("lookahead_dist"));
    }
    if (parameter_dictionary->HasKey("weight_pos")) {
        options.set_weight_pos(parameter_dictionary->GetDouble("weight_pos"));
    }
    if (parameter_dictionary->HasKey("weight_yaw")) {
        options.set_weight_yaw(parameter_dictionary->GetDouble("weight_yaw"));
    }
    if (parameter_dictionary->HasKey("weight_linear")) {
        options.set_weight_linear(
            parameter_dictionary->GetDouble("weight_linear"));
    }
    if (parameter_dictionary->HasKey("weight_angular")) {
        options.set_weight_angular(
            parameter_dictionary->GetDouble("weight_angular"));
    }
    if (parameter_dictionary->HasKey("weight_terminal_pos")) {
        options.set_weight_terminal_pos(
            parameter_dictionary->GetDouble("weight_terminal_pos"));
    }
    if (parameter_dictionary->HasKey("weight_terminal_yaw")) {
        options.set_weight_terminal_yaw(
            parameter_dictionary->GetDouble("weight_terminal_yaw"));
    }
    if (parameter_dictionary->HasKey("solver_type")) {
        options.set_solver_type(parameter_dictionary->GetString("solver_type"));
    }
    if (parameter_dictionary->HasKey("path_search_window")) {
        options.set_path_search_window(
            parameter_dictionary->GetDouble("path_search_window"));
    }
    if (parameter_dictionary->HasKey("approach_velocity_scaling_dist")) {
        options.set_approach_velocity_scaling_dist(
            parameter_dictionary->GetDouble("approach_velocity_scaling_dist"));
    }
    if (parameter_dictionary->HasKey("min_approach_linear_vel")) {
        options.set_min_approach_linear_vel(
            parameter_dictionary->GetDouble("min_approach_linear_vel"));
    }
    if (parameter_dictionary->HasKey("enable_collision_check")) {
        options.set_enable_collision_check(
            parameter_dictionary->GetBool("enable_collision_check"));
    }
    if (parameter_dictionary->HasKey("collision_cost_threshold")) {
        options.set_collision_cost_threshold(
            parameter_dictionary->GetDouble("collision_cost_threshold"));
    }
    if (parameter_dictionary->HasKey("use_footprint_collision_check")) {
        options.set_use_footprint_collision_check(
            parameter_dictionary->GetBool("use_footprint_collision_check"));
    }
    if (parameter_dictionary->HasKey("max_plan_search_dist")) {
        options.set_max_plan_search_dist(
            parameter_dictionary->GetDouble("max_plan_search_dist"));
    }
    if (parameter_dictionary->HasKey("prune_distance")) {
        options.set_prune_distance(
            parameter_dictionary->GetDouble("prune_distance"));
    }
    if (parameter_dictionary->HasKey("cgmres_horizon_divisions")) {
        options.set_cgmres_horizon_divisions(static_cast<int32_t>(
            parameter_dictionary->GetInt("cgmres_horizon_divisions")));
    }
    if (parameter_dictionary->HasKey("cgmres_k_max")) {
        options.set_cgmres_k_max(
            static_cast<int32_t>(parameter_dictionary->GetInt("cgmres_k_max")));
    }
    return ApplyDefaults(std::move(options));
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
