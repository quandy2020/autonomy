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

#include "autonomy/control/controller/teb_controller/config.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

TimedElasticBandConfig::TimedElasticBandConfig() {
    odom_topic = "odom";
    map_frame = "odom";

    // Trajectory
    trajectory.enable_auto_resize = true;
    trajectory.reference_time_step = 0.3;
    trajectory.time_step_hysteresis = 0.1;
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_via_point_separation = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.global_plan_prune_distance = 1;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.feasibility_check_pose_count = 5;
    trajectory.feasibility_check_lookahead_distance = -1;
    trajectory.publish_feedback = false;
    trajectory.min_resolution_collision_check_angular = M_PI;
    trajectory.control_lookahead_pose_count = 1;

    // Robot
    robot.max_velocity_x = 0.4;
    robot.max_velocity_x_backwards = 0.2;
    robot.max_velocity_y = 0.0;
    robot.max_angular_velocity = 0.3;
    robot.base_max_velocity_x = robot.max_velocity_x;
    robot.base_max_velocity_x_backwards = robot.max_velocity_x_backwards;
    robot.base_max_velocity_y = robot.max_velocity_y;
    robot.base_max_angular_velocity = robot.max_angular_velocity;
    robot.max_acceleration_x = 0.5;
    robot.max_acceleration_y = 0.5;
    robot.max_angular_acceleration = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.use_steering_angle_command = false;
    robot.is_footprint_dynamic = false;
    robot.use_proportional_saturation = false;
    robot.transform_tolerance = 0.5;

    // GoalTolerance
    goal_tolerance.goal_position_tolerance = 0.2;
    goal_tolerance.free_goal_velocity = false;

    // Obstacles
    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;
    obstacles.obstacle_proximity_max_velocity_ratio = 1;
    obstacles.obstacle_proximity_lower_bound = 0;
    obstacles.obstacle_proximity_upper_bound = 0.5;

    // Optimization
    optimization.inner_iteration_count = 5;
    optimization.outer_iteration_count = 4;
    optimization.optimization_activate = true;
    optimization.optimization_verbose = false;
    optimization.penalty_epsilon = 0.05;
    optimization.weight_max_velocity_x = 2;
    optimization.weight_max_velocity_y = 2;
    optimization.weight_max_angular_velocity = 1;
    optimization.weight_max_acceleration_x = 1;
    optimization.weight_max_acceleration_y = 1;
    optimization.weight_max_angular_acceleration = 1;
    optimization.weight_kinematics_non_holonomic = 1000;
    optimization.weight_kinematics_forward_drive = 1;
    optimization.weight_kinematics_turning_radius = 1;
    optimization.weight_time_optimal = 1;
    optimization.weight_shortest_path = 0;
    optimization.weight_obstacle = 50;
    optimization.weight_inflation = 0.1;
    optimization.weight_dynamic_obstacle = 50;
    optimization.weight_dynamic_obstacle_inflation = 0.1;
    optimization.weight_velocity_obstacle_ratio = 0;
    optimization.weight_via_point = 1;
    optimization.weight_preferred_rotation_direction = 50;
    optimization.weight_adapt_factor = 2.0;
    optimization.obstacle_cost_exponent = 1.0;

    // Homotopy class planner
    homotopy.enable_homotopy_class_planning = true;
    homotopy.enable_multithreading = true;
    homotopy.simple_exploration = false;
    homotopy.max_number_classes = 5;
    homotopy.selection_cost_hysteresis = 1.0;
    homotopy.selection_prefer_initial_plan = 0.95;
    homotopy.selection_obstacle_cost_scale = 100.0;
    homotopy.selection_via_point_cost_scale = 1.0;
    homotopy.selection_alternative_time_cost = false;
    homotopy.selection_dropping_probability = 0.0;
    homotopy.obstacle_keypoint_offset = 0.1;
    homotopy.obstacle_heading_threshold = 0.45;
    homotopy.roadmap_graph_sample_count = 15;
    homotopy.roadmap_graph_area_width = 6;
    homotopy.roadmap_graph_area_length_scale = 1.0;
    homotopy.homotopy_signature_prescaler = 1;
    homotopy.homotopy_signature_threshold = 0.1;
    homotopy.switching_blocking_period = 0.0;
    homotopy.via_points_all_candidates = true;
    homotopy.visualize_homotopy_graph = false;
    homotopy.visualize_with_time_as_z_axis_scale = 0.0;
    homotopy.delete_detours_backwards = true;
    homotopy.detours_orientation_tolerance = M_PI / 2.0;
    homotopy.length_start_orientation_vector = 0.4;
    homotopy.max_ratio_detours_duration_best_duration = 3.0;

    // Recovery
    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;
    recovery.divergence_detection_enable = false;
    recovery.divergence_detection_max_chi_squared = 10;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
