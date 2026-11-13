/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file options.hpp
 * @brief Fill SanD-Planner sidecar defaults.
 */

#pragma once

#include "autonomy/control/proto/policy_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Copy @p proto and fill non-positive / unset SanD-Planner knobs.
 */
inline proto::SandPlanner MakeEffectiveSandPlanner(
    const proto::PolicyConfig& proto) {
    proto::SandPlanner options = proto.sand_planner();
    if (options.control_points() <= 0) {
        const int horizon = proto.diffusion().horizon();
        options.set_control_points(horizon > 0 ? horizon : 8);
    }
    if (options.waypoint_count() <= 0) {
        options.set_waypoint_count(32);
    }
    if (options.depth_history() <= 0) {
        options.set_depth_history(4);
    }
    if (options.depth_height() <= 0) {
        options.set_depth_height(168);
    }
    if (options.depth_width() <= 0) {
        options.set_depth_width(224);
    }
    if (options.source_width() <= 0.0f) {
        options.set_source_width(640.0f);
    }
    if (options.source_height() <= 0.0f) {
        options.set_source_height(480.0f);
    }
    if (options.focal_x() <= 0.0f) {
        options.set_focal_x(389.551f);
    }
    if (options.focal_y() <= 0.0f) {
        options.set_focal_y(options.focal_x());
    }
    if (options.principal_x() <= 0.0f) {
        options.set_principal_x(324.211f);
    }
    if (options.principal_y() <= 0.0f) {
        options.set_principal_y(235.656f);
    }
    if (options.max_depth() <= 0.0f) {
        options.set_max_depth(8.0f);
    }
    if (options.safety_margin() <= 0.0f) {
        options.set_safety_margin(0.25f);
    }
    if (options.discount() <= 0.0f || options.discount() >= 1.0f) {
        options.set_discount(0.75f);
    }
    if (options.lambda_safety() <= 0.0f) {
        options.set_lambda_safety(10000.0f);
    }
    if (options.lambda_length() <= 0.0f) {
        options.set_lambda_length(0.01f);
    }
    if (options.lambda_goal() <= 0.0f) {
        options.set_lambda_goal(0.0001f);
    }
    if (options.execute_speed() <= 0.0f) {
        options.set_execute_speed(0.5f);
    }
    if (options.pixel_stride() <= 0) {
        options.set_pixel_stride(4);
    }
    if (options.esdf_resolution() <= 0.0f) {
        options.set_esdf_resolution(0.05f);
    }
    if (options.esdf_size_x() <= 0.0f || options.esdf_size_y() <= 0.0f ||
        options.esdf_size_z() <= 0.0f) {
        options.set_esdf_origin_x(-0.5f);
        options.set_esdf_origin_y(-2.0f);
        options.set_esdf_origin_z(-1.5f);
        options.set_esdf_size_x(5.5f);
        options.set_esdf_size_y(4.0f);
        options.set_esdf_size_z(3.0f);
    }
    if (options.preferred_clearance() <= 0.0f) {
        options.set_preferred_clearance(0.6f);
    }
    if (options.clearance_samples() <= 0) {
        options.set_clearance_samples(40);
    }
    return options;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
