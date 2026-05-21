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

#include "autonomy/control/controller/tdmpc_controller/parameter_options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {

proto::TdmpcControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::TdmpcControllerOptions options;
    if (!parameter_dictionary) {
        return options;
    }

    auto load_double = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(parameter_dictionary->GetDouble(key));
        }
    };
    auto load_string = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(parameter_dictionary->GetString(key));
        }
    };
    auto load_bool = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(parameter_dictionary->GetBool(key));
        }
    };
    auto load_int = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(static_cast<int32_t>(parameter_dictionary->GetDouble(key)));
        }
    };

    load_string("kinematic_model", [&](const std::string& v) {
        options.set_kinematic_model(v);
    });
    load_double("track_width", [&](double v) { options.set_track_width(v); });
    load_string("quadruped_gait", [&](const std::string& v) {
        options.set_quadruped_gait(v);
    });
    load_double("max_lateral_velocity",
                [&](double v) { options.set_max_lateral_velocity(v); });
    load_double("min_turn_radius",
                [&](double v) { options.set_min_turn_radius(v); });
    load_double("r_vy", [&](double v) { options.set_r_vy(v); });
    load_double("transform_tolerance",
                [&](double v) { options.set_transform_tolerance(v); });
    load_double("max_robot_pose_search_dist", [&](double v) {
        options.set_max_robot_pose_search_dist(v);
    });
    load_int("horizon", [&](int32_t v) { options.set_horizon(v); });
    load_double("dt", [&](double v) { options.set_dt(v); });
    load_double("q_contour", [&](double v) { options.set_q_contour(v); });
    load_double("q_lag", [&](double v) { options.set_q_lag(v); });
    load_double("q_yaw", [&](double v) { options.set_q_yaw(v); });
    load_double("q_velocity", [&](double v) { options.set_q_velocity(v); });
    load_double("q_contour_terminal",
                [&](double v) { options.set_q_contour_terminal(v); });
    load_double("q_lag_terminal", [&](double v) { options.set_q_lag_terminal(v); });
    load_double("q_yaw_terminal", [&](double v) { options.set_q_yaw_terminal(v); });
    load_double("r_v", [&](double v) { options.set_r_v(v); });
    load_double("r_omega", [&](double v) { options.set_r_omega(v); });
    load_double("r_du", [&](double v) { options.set_r_du(v); });
    load_double("v_min", [&](double v) { options.set_v_min(v); });
    load_double("v_max", [&](double v) { options.set_v_max(v); });
    load_double("omega_min", [&](double v) { options.set_omega_min(v); });
    load_double("omega_max", [&](double v) { options.set_omega_max(v); });
    load_double("reference_velocity",
                [&](double v) { options.set_reference_velocity(v); });
    load_int("num_topology_candidates", [&](int32_t v) {
        options.set_num_topology_candidates(v);
    });
    load_double("topology_lateral_spacing", [&](double v) {
        options.set_topology_lateral_spacing(v);
    });
    load_bool("enable_baseline_topology", [&](bool v) {
        options.set_enable_baseline_topology(v);
    });
    load_double("q_obstacle", [&](double v) { options.set_q_obstacle(v); });
    load_double("obstacle_cost_threshold", [&](double v) {
        options.set_obstacle_cost_threshold(v);
    });
    load_bool("enable_costmap_constraints", [&](bool v) {
        options.set_enable_costmap_constraints(v);
    });
    load_int("max_iterations", [&](int32_t v) { options.set_max_iterations(v); });
    load_double("cost_tolerance", [&](double v) { options.set_cost_tolerance(v); });
    load_double("ipopt_max_cpu_time",
                [&](double v) { options.set_ipopt_max_cpu_time(v); });
    load_bool("use_fallback_on_failure", [&](bool v) {
        options.set_use_fallback_on_failure(v);
    });
    load_double("slowdown_radius", [&](double v) { options.set_slowdown_radius(v); });
    load_double("yaw_rate_kp", [&](double v) { options.set_yaw_rate_kp(v); });
    load_bool("allow_backward", [&](bool v) { options.set_allow_backward(v); });

    return options;
}

}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
