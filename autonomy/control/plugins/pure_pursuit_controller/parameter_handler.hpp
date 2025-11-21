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

#pragma once 

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <limits>
#include <utility>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/costmap_2d/utils/odometry_utils.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace plugins {

struct Parameters
{
    double desired_linear_vel, base_desired_linear_vel;
    double lookahead_dist;
    double rotate_to_heading_angular_vel;
    double max_lookahead_dist;
    double min_lookahead_dist;
    double lookahead_time;
    bool use_velocity_scaled_lookahead_dist;
    double min_approach_linear_velocity;
    double approach_velocity_scaling_dist;
    double max_allowed_time_to_collision_up_to_carrot;
    bool use_regulated_linear_velocity_scaling;
    bool use_cost_regulated_linear_velocity_scaling;
    double cost_scaling_dist;
    double cost_scaling_gain;
    double inflation_cost_scaling_factor;
    double regulated_linear_scaling_min_radius;
    double regulated_linear_scaling_min_speed;
    bool use_fixed_curvature_lookahead;
    double curvature_lookahead_dist;
    bool use_rotate_to_heading;
    double max_angular_accel;
    bool use_cancel_deceleration;
    double cancel_deceleration;
    double rotate_to_heading_min_angle;
    bool allow_reversing;
    double max_robot_pose_search_dist;
    bool interpolate_curvature_after_goal;
    bool use_collision_detection;
    double transform_tolerance;
};

/**
 * @class nav2_regulated_pure_pursuit_controller::ParameterHandler
 * @brief Handles parameters and dynamic parameters for RPP
 */
class ParameterHandler
{
public:
    /**
     * @brief Constructor for nav2_regulated_pure_pursuit_controller::ParameterHandler
     */
    ParameterHandler(std::string& plugin_name, const double costmap_size_x);

    std::mutex& getMutex() {return mutex_;}

    Parameters* getParams() {return &params_;}

protected:
    // Dynamic parameters handler
    std::mutex mutex_;
    Parameters params_;
    std::string plugin_name_;
};

}  // namespace plugins
}  // namespace control
}  // namespace autonomy