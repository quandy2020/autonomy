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

#include "autonomy/control/controller/pure_pursuit_controller/parameter_options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace pure_pursuit_controller {

proto::PurePursuitControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::PurePursuitControllerOptions options;

    if (!parameter_dictionary) {
        return options;
    }

    // Start from nav2-like defaults, then override from Lua.
    // This is important because proto3 scalar fields don't have presence by
    // default.
    options.set_desired_linear_vel(0.5);
    options.set_lookahead_dist(0.6);
    options.set_min_lookahead_dist(0.3);
    options.set_max_lookahead_dist(0.9);
    options.set_lookahead_time(1.5);
    options.set_use_velocity_scaled_lookahead_dist(false);
    options.set_rotate_to_heading_angular_vel(1.8);
    options.set_rotate_to_heading_min_angle(0.785);
    options.set_max_angular_accel(3.2);
    options.set_use_rotate_to_heading(true);
    options.set_use_cancel_deceleration(false);
    options.set_cancel_deceleration(3.2);
    options.set_allow_reversing(false);
    options.set_max_robot_pose_search_dist(
        0.0);  // filled by ParameterHandler based on costmap size
    options.set_use_fixed_curvature_lookahead(false);
    options.set_curvature_lookahead_dist(0.6);
    options.set_interpolate_curvature_after_goal(false);
    options.set_use_regulated_linear_velocity_scaling(true);
    options.set_regulated_linear_scaling_min_radius(0.90);
    options.set_regulated_linear_scaling_min_speed(0.25);
    options.set_use_cost_regulated_linear_velocity_scaling(true);
    options.set_cost_scaling_dist(0.6);
    options.set_cost_scaling_gain(1.0);
    options.set_inflation_cost_scaling_factor(3.0);
    options.set_min_approach_linear_velocity(0.05);
    options.set_approach_velocity_scaling_dist(0.6);
    options.set_use_collision_detection(true);
    options.set_max_allowed_time_to_collision_up_to_carrot(1.0);
    options.set_transform_tolerance(0.1);
    options.set_stateful(true);

    if (parameter_dictionary->HasKey("desired_linear_vel")) {
        options.set_desired_linear_vel(
            parameter_dictionary->GetDouble("desired_linear_vel"));
    }
    if (parameter_dictionary->HasKey("lookahead_dist")) {
        options.set_lookahead_dist(
            parameter_dictionary->GetDouble("lookahead_dist"));
    }
    if (parameter_dictionary->HasKey("min_lookahead_dist")) {
        options.set_min_lookahead_dist(
            parameter_dictionary->GetDouble("min_lookahead_dist"));
    }
    if (parameter_dictionary->HasKey("max_lookahead_dist")) {
        options.set_max_lookahead_dist(
            parameter_dictionary->GetDouble("max_lookahead_dist"));
    }
    if (parameter_dictionary->HasKey("lookahead_time")) {
        options.set_lookahead_time(
            parameter_dictionary->GetDouble("lookahead_time"));
    }
    if (parameter_dictionary->HasKey("use_velocity_scaled_lookahead_dist")) {
        options.set_use_velocity_scaled_lookahead_dist(
            parameter_dictionary->GetBool(
                "use_velocity_scaled_lookahead_dist"));
    }

    if (parameter_dictionary->HasKey("rotate_to_heading_angular_vel")) {
        options.set_rotate_to_heading_angular_vel(
            parameter_dictionary->GetDouble("rotate_to_heading_angular_vel"));
    }
    if (parameter_dictionary->HasKey("rotate_to_heading_min_angle")) {
        options.set_rotate_to_heading_min_angle(
            parameter_dictionary->GetDouble("rotate_to_heading_min_angle"));
    }
    if (parameter_dictionary->HasKey("max_angular_accel")) {
        options.set_max_angular_accel(
            parameter_dictionary->GetDouble("max_angular_accel"));
    }
    if (parameter_dictionary->HasKey("use_rotate_to_heading")) {
        options.set_use_rotate_to_heading(
            parameter_dictionary->GetBool("use_rotate_to_heading"));
    }

    if (parameter_dictionary->HasKey("use_cancel_deceleration")) {
        options.set_use_cancel_deceleration(
            parameter_dictionary->GetBool("use_cancel_deceleration"));
    }
    if (parameter_dictionary->HasKey("cancel_deceleration")) {
        options.set_cancel_deceleration(
            parameter_dictionary->GetDouble("cancel_deceleration"));
    }

    if (parameter_dictionary->HasKey("allow_reversing")) {
        options.set_allow_reversing(
            parameter_dictionary->GetBool("allow_reversing"));
    }
    if (parameter_dictionary->HasKey("max_robot_pose_search_dist")) {
        options.set_max_robot_pose_search_dist(
            parameter_dictionary->GetDouble("max_robot_pose_search_dist"));
    }

    if (parameter_dictionary->HasKey("use_fixed_curvature_lookahead")) {
        options.set_use_fixed_curvature_lookahead(
            parameter_dictionary->GetBool("use_fixed_curvature_lookahead"));
    }
    if (parameter_dictionary->HasKey("curvature_lookahead_dist")) {
        options.set_curvature_lookahead_dist(
            parameter_dictionary->GetDouble("curvature_lookahead_dist"));
    }
    if (parameter_dictionary->HasKey("interpolate_curvature_after_goal")) {
        options.set_interpolate_curvature_after_goal(
            parameter_dictionary->GetBool("interpolate_curvature_after_goal"));
    }

    if (parameter_dictionary->HasKey("use_regulated_linear_velocity_scaling")) {
        options.set_use_regulated_linear_velocity_scaling(
            parameter_dictionary->GetBool(
                "use_regulated_linear_velocity_scaling"));
    }
    if (parameter_dictionary->HasKey("regulated_linear_scaling_min_radius")) {
        options.set_regulated_linear_scaling_min_radius(
            parameter_dictionary->GetDouble(
                "regulated_linear_scaling_min_radius"));
    }
    if (parameter_dictionary->HasKey("regulated_linear_scaling_min_speed")) {
        options.set_regulated_linear_scaling_min_speed(
            parameter_dictionary->GetDouble(
                "regulated_linear_scaling_min_speed"));
    }

    if (parameter_dictionary->HasKey(
            "use_cost_regulated_linear_velocity_scaling")) {
        options.set_use_cost_regulated_linear_velocity_scaling(
            parameter_dictionary->GetBool(
                "use_cost_regulated_linear_velocity_scaling"));
    }
    if (parameter_dictionary->HasKey("cost_scaling_dist")) {
        options.set_cost_scaling_dist(
            parameter_dictionary->GetDouble("cost_scaling_dist"));
    }
    if (parameter_dictionary->HasKey("cost_scaling_gain")) {
        options.set_cost_scaling_gain(
            parameter_dictionary->GetDouble("cost_scaling_gain"));
    }
    if (parameter_dictionary->HasKey("inflation_cost_scaling_factor")) {
        options.set_inflation_cost_scaling_factor(
            parameter_dictionary->GetDouble("inflation_cost_scaling_factor"));
    }

    if (parameter_dictionary->HasKey("min_approach_linear_velocity")) {
        options.set_min_approach_linear_velocity(
            parameter_dictionary->GetDouble("min_approach_linear_velocity"));
    }
    if (parameter_dictionary->HasKey("approach_velocity_scaling_dist")) {
        options.set_approach_velocity_scaling_dist(
            parameter_dictionary->GetDouble("approach_velocity_scaling_dist"));
    }

    if (parameter_dictionary->HasKey("use_collision_detection")) {
        options.set_use_collision_detection(
            parameter_dictionary->GetBool("use_collision_detection"));
    }
    if (parameter_dictionary->HasKey(
            "max_allowed_time_to_collision_up_to_carrot")) {
        options.set_max_allowed_time_to_collision_up_to_carrot(
            parameter_dictionary->GetDouble(
                "max_allowed_time_to_collision_up_to_carrot"));
    }

    if (parameter_dictionary->HasKey("transform_tolerance")) {
        options.set_transform_tolerance(
            parameter_dictionary->GetDouble("transform_tolerance"));
    }
    if (parameter_dictionary->HasKey("stateful")) {
        options.set_stateful(parameter_dictionary->GetBool("stateful"));
    }

    return options;
}

}  // namespace pure_pursuit_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
