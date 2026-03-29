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

#include "autonomy/control/controller/pure_pursuit_controller/parameter_handler.hpp"

#include <limits>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace pure_pursuit_controller {

namespace {

proto::PurePursuitControllerOptions DefaultOptions() {
    proto::PurePursuitControllerOptions o;
    // nav2 defaults (see nav2_regulated_pure_pursuit_controller
    // ParameterHandler)
    o.set_desired_linear_vel(0.5);
    o.set_lookahead_dist(0.6);
    o.set_min_lookahead_dist(0.3);
    o.set_max_lookahead_dist(0.9);
    o.set_lookahead_time(1.5);
    o.set_rotate_to_heading_angular_vel(1.8);
    o.set_transform_tolerance(0.1);
    o.set_use_velocity_scaled_lookahead_dist(false);
    o.set_min_approach_linear_velocity(0.05);
    o.set_approach_velocity_scaling_dist(0.6);
    o.set_max_allowed_time_to_collision_up_to_carrot(1.0);
    o.set_use_regulated_linear_velocity_scaling(true);
    o.set_use_cost_regulated_linear_velocity_scaling(true);
    o.set_cost_scaling_dist(0.6);
    o.set_cost_scaling_gain(1.0);
    o.set_inflation_cost_scaling_factor(3.0);
    o.set_regulated_linear_scaling_min_radius(0.90);
    o.set_regulated_linear_scaling_min_speed(0.25);
    o.set_use_fixed_curvature_lookahead(false);
    o.set_curvature_lookahead_dist(0.6);
    o.set_use_rotate_to_heading(true);
    o.set_rotate_to_heading_min_angle(0.785);
    o.set_max_angular_accel(3.2);
    o.set_use_cancel_deceleration(false);
    o.set_cancel_deceleration(3.2);
    o.set_allow_reversing(false);
    // max_robot_pose_search_dist depends on costmap size, handled later
    o.set_interpolate_curvature_after_goal(false);
    o.set_use_collision_detection(true);
    o.set_stateful(true);
    return o;
}

bool MergeWithDefaultsIfEmpty(const proto::PurePursuitControllerOptions& in,
                              proto::PurePursuitControllerOptions* out) {
    if (!out) {
        return false;
    }
    // If this message is empty (all fields default), treat it as "unset" and
    // use defaults.
    if (in.ByteSizeLong() == 0) {
        *out = DefaultOptions();
        return true;
    }
    *out = in;
    return true;
}

}  // namespace

ParameterHandler::ParameterHandler(
    const proto::PurePursuitControllerOptions& options,
    const double costmap_size_x) {
    std::string error;
    if (!Update(options, costmap_size_x, &error)) {
        AWARN << "PurePursuit parameter init adjusted: " << error;
    }
}

bool ParameterHandler::Update(
    const proto::PurePursuitControllerOptions& options,
    const double costmap_size_x, std::string* error) {
    std::lock_guard<std::mutex> lock(mutex_);

    proto::PurePursuitControllerOptions effective;
    MergeWithDefaultsIfEmpty(options, &effective);

    // Fill in defaults that depend on runtime context.
    if (effective.max_robot_pose_search_dist() == 0.0) {
        effective.set_max_robot_pose_search_dist(costmap_size_x / 2.0);
    }

    // Validate / normalize options (nav2-compatible behavior).
    std::string local_error;
    if (effective.inflation_cost_scaling_factor() <= 0.0) {
        local_error +=
            "inflation_cost_scaling_factor<=0, disabling cost regulated "
            "scaling; ";
        effective.set_use_cost_regulated_linear_velocity_scaling(false);
    }

    if (effective.max_robot_pose_search_dist() < 0.0) {
        local_error += "max_robot_pose_search_dist<0, setting to max; ";
        effective.set_max_robot_pose_search_dist(
            std::numeric_limits<double>::max());
    }

    if (!effective.use_fixed_curvature_lookahead() &&
        effective.interpolate_curvature_after_goal()) {
        local_error +=
            "interpolate_curvature_after_goal requires "
            "use_fixed_curvature_lookahead, disabling; ";
        effective.set_interpolate_curvature_after_goal(false);
    }

    // nav2 rejects: allow_reversing=true and use_rotate_to_heading=true
    // simultaneously. Here we resolve by disabling allow_reversing.
    if (effective.use_rotate_to_heading() && effective.allow_reversing()) {
        local_error +=
            "use_rotate_to_heading && allow_reversing conflict, disabling "
            "allow_reversing; ";
        effective.set_allow_reversing(false);
    }

    // Apply to stored proto and runtime params.
    options_ = effective;
    base_desired_linear_vel_ = options_.desired_linear_vel();

    if (error) {
        *error = local_error;
    }
    return local_error.empty();
}

}  // namespace pure_pursuit_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy