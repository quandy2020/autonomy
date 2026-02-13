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

#include "autonomy/control/controller/graceful_controller/parameter_options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace graceful_controller {

proto::GracefulControllerOptions LoadOptions(::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::GracefulControllerOptions options;

    if (!parameter_dictionary) {
        return options;
    }

    // Load all graceful controller parameters
    if (parameter_dictionary->HasKey("transform_tolerance")) {
        options.set_transform_tolerance(parameter_dictionary->GetDouble("transform_tolerance"));
    }
    if (parameter_dictionary->HasKey("max_lookahead")) {
        options.set_max_lookahead(parameter_dictionary->GetDouble("max_lookahead"));
    }
    if (parameter_dictionary->HasKey("min_lookahead")) {
        options.set_min_lookahead(parameter_dictionary->GetDouble("min_lookahead"));
    }
    if (parameter_dictionary->HasKey("max_robot_pose_search_dist")) {
        options.set_max_robot_pose_search_dist(parameter_dictionary->GetDouble("max_robot_pose_search_dist"));
    }
    if (parameter_dictionary->HasKey("k_phi")) {
        options.set_k_phi(parameter_dictionary->GetDouble("k_phi"));
    }
    if (parameter_dictionary->HasKey("k_delta")) {
        options.set_k_delta(parameter_dictionary->GetDouble("k_delta"));
    }
    if (parameter_dictionary->HasKey("beta")) {
        options.set_beta(parameter_dictionary->GetDouble("beta"));
    }
    if (parameter_dictionary->HasKey("lambda")) {
        options.set_lambda(parameter_dictionary->GetDouble("lambda"));
    }
    if (parameter_dictionary->HasKey("v_linear_min")) {
        options.set_v_linear_min(parameter_dictionary->GetDouble("v_linear_min"));
    }
    if (parameter_dictionary->HasKey("v_linear_max")) {
        options.set_v_linear_max(parameter_dictionary->GetDouble("v_linear_max"));
    }
    if (parameter_dictionary->HasKey("v_angular_max")) {
        options.set_v_angular_max(parameter_dictionary->GetDouble("v_angular_max"));
    }
    if (parameter_dictionary->HasKey("v_angular_min_in_place")) {
        options.set_v_angular_min_in_place(parameter_dictionary->GetDouble("v_angular_min_in_place"));
    }
    if (parameter_dictionary->HasKey("slowdown_radius")) {
        options.set_slowdown_radius(parameter_dictionary->GetDouble("slowdown_radius"));
    }
    if (parameter_dictionary->HasKey("initial_rotation")) {
        options.set_initial_rotation(parameter_dictionary->GetBool("initial_rotation"));
    }
    if (parameter_dictionary->HasKey("initial_rotation_tolerance")) {
        options.set_initial_rotation_tolerance(parameter_dictionary->GetDouble("initial_rotation_tolerance"));
    }
    if (parameter_dictionary->HasKey("prefer_final_rotation")) {
        options.set_prefer_final_rotation(parameter_dictionary->GetBool("prefer_final_rotation"));
    }
    if (parameter_dictionary->HasKey("rotation_scaling_factor")) {
        options.set_rotation_scaling_factor(parameter_dictionary->GetDouble("rotation_scaling_factor"));
    }
    if (parameter_dictionary->HasKey("allow_backward")) {
        options.set_allow_backward(parameter_dictionary->GetBool("allow_backward"));
    }
    if (parameter_dictionary->HasKey("use_collision_detection")) {
        options.set_use_collision_detection(parameter_dictionary->GetBool("use_collision_detection"));
    }
    if (parameter_dictionary->HasKey("in_place_collision_resolution")) {
        options.set_in_place_collision_resolution(parameter_dictionary->GetDouble("in_place_collision_resolution"));
    }

    return options;
}

}  // namespace graceful_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
