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

#include "autonomy/control/controller/nmpc_controller/parameter_options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {

proto::NmpcControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::NmpcControllerOptions options;
    if (!parameter_dictionary) {
        return options;
    }

    if (parameter_dictionary->HasKey("transform_tolerance")) {
        options.set_transform_tolerance(
            parameter_dictionary->GetDouble("transform_tolerance"));
    }
    if (parameter_dictionary->HasKey("max_robot_pose_search_dist")) {
        options.set_max_robot_pose_search_dist(
            parameter_dictionary->GetDouble("max_robot_pose_search_dist"));
    }
    if (parameter_dictionary->HasKey("horizon")) {
        options.set_horizon(
            static_cast<int32_t>(parameter_dictionary->GetDouble("horizon")));
    }
    if (parameter_dictionary->HasKey("dt")) {
        options.set_dt(parameter_dictionary->GetDouble("dt"));
    }
    if (parameter_dictionary->HasKey("q_x")) {
        options.set_q_x(parameter_dictionary->GetDouble("q_x"));
    }
    if (parameter_dictionary->HasKey("q_y")) {
        options.set_q_y(parameter_dictionary->GetDouble("q_y"));
    }
    if (parameter_dictionary->HasKey("q_yaw")) {
        options.set_q_yaw(parameter_dictionary->GetDouble("q_yaw"));
    }
    if (parameter_dictionary->HasKey("q_x_terminal")) {
        options.set_q_x_terminal(parameter_dictionary->GetDouble("q_x_terminal"));
    }
    if (parameter_dictionary->HasKey("q_y_terminal")) {
        options.set_q_y_terminal(parameter_dictionary->GetDouble("q_y_terminal"));
    }
    if (parameter_dictionary->HasKey("q_yaw_terminal")) {
        options.set_q_yaw_terminal(
            parameter_dictionary->GetDouble("q_yaw_terminal"));
    }
    if (parameter_dictionary->HasKey("r_v")) {
        options.set_r_v(parameter_dictionary->GetDouble("r_v"));
    }
    if (parameter_dictionary->HasKey("r_omega")) {
        options.set_r_omega(parameter_dictionary->GetDouble("r_omega"));
    }
    if (parameter_dictionary->HasKey("v_min")) {
        options.set_v_min(parameter_dictionary->GetDouble("v_min"));
    }
    if (parameter_dictionary->HasKey("v_max")) {
        options.set_v_max(parameter_dictionary->GetDouble("v_max"));
    }
    if (parameter_dictionary->HasKey("omega_min")) {
        options.set_omega_min(parameter_dictionary->GetDouble("omega_min"));
    }
    if (parameter_dictionary->HasKey("omega_max")) {
        options.set_omega_max(parameter_dictionary->GetDouble("omega_max"));
    }
    if (parameter_dictionary->HasKey("reference_velocity")) {
        options.set_reference_velocity(
            parameter_dictionary->GetDouble("reference_velocity"));
    }
    if (parameter_dictionary->HasKey("max_iterations")) {
        options.set_max_iterations(static_cast<int32_t>(
            parameter_dictionary->GetDouble("max_iterations")));
    }
    if (parameter_dictionary->HasKey("step_size")) {
        options.set_step_size(parameter_dictionary->GetDouble("step_size"));
    }
    if (parameter_dictionary->HasKey("gradient_epsilon")) {
        options.set_gradient_epsilon(
            parameter_dictionary->GetDouble("gradient_epsilon"));
    }
    if (parameter_dictionary->HasKey("cost_tolerance")) {
        options.set_cost_tolerance(
            parameter_dictionary->GetDouble("cost_tolerance"));
    }
    if (parameter_dictionary->HasKey("kinematic_model")) {
        options.set_kinematic_model(
            parameter_dictionary->GetString("kinematic_model"));
    }
    if (parameter_dictionary->HasKey("track_width")) {
        options.set_track_width(parameter_dictionary->GetDouble("track_width"));
    }
    if (parameter_dictionary->HasKey("quadruped_gait")) {
        options.set_quadruped_gait(
            parameter_dictionary->GetString("quadruped_gait"));
    }
    if (parameter_dictionary->HasKey("max_lateral_velocity")) {
        options.set_max_lateral_velocity(
            parameter_dictionary->GetDouble("max_lateral_velocity"));
    }
    if (parameter_dictionary->HasKey("min_turn_radius")) {
        options.set_min_turn_radius(
            parameter_dictionary->GetDouble("min_turn_radius"));
    }
    if (parameter_dictionary->HasKey("r_vy")) {
        options.set_r_vy(parameter_dictionary->GetDouble("r_vy"));
    }
    if (parameter_dictionary->HasKey("slowdown_radius")) {
        options.set_slowdown_radius(
            parameter_dictionary->GetDouble("slowdown_radius"));
    }
    if (parameter_dictionary->HasKey("ipopt_max_cpu_time")) {
        options.set_ipopt_max_cpu_time(
            parameter_dictionary->GetDouble("ipopt_max_cpu_time"));
    }
    if (parameter_dictionary->HasKey("use_fallback_on_failure")) {
        options.set_use_fallback_on_failure(
            parameter_dictionary->GetBool("use_fallback_on_failure"));
    }
    if (parameter_dictionary->HasKey("yaw_rate_kp")) {
        options.set_yaw_rate_kp(parameter_dictionary->GetDouble("yaw_rate_kp"));
    }
    if (parameter_dictionary->HasKey("allow_backward")) {
        options.set_allow_backward(
            parameter_dictionary->GetBool("allow_backward"));
    }
    if (parameter_dictionary->HasKey("r_du")) {
        options.set_r_du(parameter_dictionary->GetDouble("r_du"));
    }
    return options;
}

}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
