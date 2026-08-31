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
 * @file parameter_defaults.cpp
 * @brief Default values for NMPCControllerOptions
 */

#include "autonomy/control/controller/nmpc_controller/parameter_defaults.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

proto::NMPCControllerOptions ApplyDefaults(proto::NMPCControllerOptions options) {
    if (options.transform_tolerance() == 0.0) {
        options.set_transform_tolerance(0.1);
    }
    if (options.model_dt() == 0.0) {
        options.set_model_dt(0.05);
    }
    if (options.horizon_steps() <= 0) {
        options.set_horizon_steps(40);
    }
    if (options.max_solver_iter() <= 0) {
        options.set_max_solver_iter(30);
    }
    if (options.max_linear_vel() == 0.0) {
        options.set_max_linear_vel(0.5);
    }
    if (options.min_linear_vel() == 0.0) {
        options.set_min_linear_vel(-0.2);
    }
    if (options.max_angular_vel() == 0.0) {
        options.set_max_angular_vel(1.5);
    }
    if (options.lookahead_dist() == 0.0) {
        options.set_lookahead_dist(0.6);
    }
    if (options.weight_pos() == 0.0) {
        options.set_weight_pos(10.0);
    }
    if (options.weight_yaw() == 0.0) {
        options.set_weight_yaw(5.0);
    }
    if (options.weight_linear() == 0.0) {
        options.set_weight_linear(0.1);
    }
    if (options.weight_angular() == 0.0) {
        options.set_weight_angular(0.05);
    }
    if (options.weight_terminal_pos() == 0.0) {
        options.set_weight_terminal_pos(50.0);
    }
    if (options.weight_terminal_yaw() == 0.0) {
        options.set_weight_terminal_yaw(20.0);
    }
    if (options.solver_type().empty()) {
        options.set_solver_type("ddp");
    }
    if (options.path_search_window() == 0.0) {
        options.set_path_search_window(2.0);
    }
    if (options.approach_velocity_scaling_dist() == 0.0) {
        options.set_approach_velocity_scaling_dist(0.8);
    }
    if (options.min_approach_linear_vel() == 0.0) {
        options.set_min_approach_linear_vel(0.05);
    }
    if (options.collision_cost_threshold() <= 0.0) {
        options.set_collision_cost_threshold(253.0);
    }
    if (options.max_plan_search_dist() == 0.0) {
        options.set_max_plan_search_dist(5.0);
    }
    if (options.prune_distance() == 0.0) {
        options.set_prune_distance(1.5);
    }
    if (options.cgmres_horizon_divisions() <= 0) {
        options.set_cgmres_horizon_divisions(25);
    }
    if (options.cgmres_k_max() <= 0) {
        options.set_cgmres_k_max(5);
    }
    return options;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
