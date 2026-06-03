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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "autolink/macros.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/** @brief Preferred initial rotation direction (prefer-rotdir cost). */
enum class PreferredRotationDirection { left, none, right };

/**
 * @brief Trajectory related parameters
 */
struct TrajectoryOptions {
    /**
     * Define TrajectoryOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(TrajectoryOptions);

    // Enable automatic resizing of the trajectory w.r.t to the temporal
    // resolution (recommended)
    bool enable_auto_resize;

    // Desired temporal resolution of the trajectory (should be in the magniture
    // of the underlying control rate)
    double reference_time_step;

    // Hysteresis for automatic resizing depending on the current temporal
    // resolution (dt): usually 10% of reference_time_step
    double time_step_hysteresis;

    // Minimum number of samples (should be always greater than 2)
    int min_samples;

    // Maximum number of samples; Warning: if too small the
    // discretization/resolution might not be sufficient for the given robot
    // model or obstacle avoidance does not work anymore.
    int max_samples;

    // Overwrite orientation of local subgoals provided by the global planner
    bool global_plan_overwrite_orientation;

    // If true, the underlying trajectories might be initialized with backwards
    // motions in case the goal is behind the start within the local costmap
    // (this is only recommended if the robot is equipped with rear sensors)
    bool allow_init_with_backwards_motion;

    // Min. separation between each two consecutive via-points extracted from
    // the global plan (if negative: disabled)
    double global_plan_via_point_separation;

    // If true, the planner adheres to the order of via-points in the storage
    // container
    bool via_points_ordered;

    // Specify maximum length (cumulative Euclidean distances) of the subset of
    // the global plan taken into account for optimization [if <=0: disabled;
    // the length is also bounded by the local costmap size!]
    double max_global_plan_lookahead_dist;

    // Distance between robot and via_points of global plan which is used for
    // pruning
    double global_plan_prune_distance;

    // If true, the planner uses the exact arc length in velocity, acceleration
    // and turning rate computations [-> increased cpu time], otherwise the
    // euclidean approximation is used.
    bool exact_arc_length;

    // Reinitialize the trajectory if a previous goal is updated with a
    // seperation of more than the specified value in meters (skip hot-starting)
    double force_reinit_new_goal_dist;

    // Reinitialize the trajectory if a previous goal is updated with an angular
    // difference of more than the specified value in radians (skip
    // hot-starting)
    double force_reinit_new_goal_angular;

    // Specify up to which pose (under the feasibility_check_lookahead_distance)
    // on the predicted plan the feasibility should be checked each sampling
    // interval; if -1, all poses up to feasibility_check_lookahead_distance are
    // checked.
    int feasibility_check_pose_count;

    // Specify up to which distance (and with an index below
    // feasibility_check_pose_count) from the robot the feasibility should be
    // checked each sampling interval; if -1, all poses up to
    // feasibility_check_pose_count are checked.
    double feasibility_check_lookahead_distance;

    // Publish planner feedback containing the full trajectory and a list of
    // active obstacles (should be enabled only for evaluation or debugging
    bool publish_feedback;

    // Min angular resolution used during the costmap collision check. If not
    // respected, intermediate samples are added. [rad]
    double min_resolution_collision_check_angular;

    // Index of the pose used to extract the velocity command
    int control_lookahead_pose_count;
};

/**
 * @brief Robot related parameters
 */
struct RobotOptions {
    /**
     * Define RobotOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(RobotOptions);

    // Maximum translational velocity of the robot before speed limit is applied
    double base_max_velocity_x;

    // Maximum translational velocity of the robot for driving backwards before
    // speed limit is applied
    double base_max_velocity_x_backwards;

    // Maximum strafing velocity of the robot (should be zero for non-holonomic
    // robots!) before speed limit is applied
    double base_max_velocity_y;

    // Maximum angular velocity of the robot before speed limit is applied
    double base_max_angular_velocity;

    // Maximum translational velocity of the robot
    double max_velocity_x;

    // Maximum translational velocity of the robot for driving backwards
    double max_velocity_x_backwards;

    // Maximum strafing velocity of the robot (should be zero for non-holonomic
    double max_velocity_y;

    // Maximum angular velocity of the robot
    double max_angular_velocity;

    // Maximum translational acceleration of the robot
    double max_acceleration_x;

    // Maximum strafing acceleration of the robot
    double max_acceleration_y;

    // Maximum angular acceleration of the robot
    double max_angular_acceleration;

    // Minimum turning radius of a carlike robot (diff-drive robot: zero);
    double min_turning_radius;

    // The distance between the drive shaft and steering axle (only required for
    // a carlike robot with 'use_steering_angle_command' enabled); The value might
    // be negative for back-wheeled robots!
    double wheelbase;

    // Substitute the rotational velocity in the commanded velocity message by
    // the corresponding steering angle (check 'axles_distance')
    bool use_steering_angle_command;

    // If true, updated the footprint before checking trajectory feasibility
    bool is_footprint_dynamic;

    // If true, reduce all twists components (linear x and y, and angular z)
    // proportionally if any exceed its corresponding bounds, instead of
    // saturating each one individually
    bool use_proportional_saturation;

    // Tolerance when querying the TF Tree for a transformation (seconds)
    double transform_tolerance;
};

/**
 * @brief Goal tolerance parameters
 */
struct GoalToleranceOptions {
    /**
     * Define GoalToleranceOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(GoalToleranceOptions);

    // Allowed final euclidean distance to the goal position
    double goal_position_tolerance;

    // Allow the robot's velocity to be nonzero (usally max_vel) for planning
    // purposes
    bool free_goal_velocity;
};

/**
 * @brief Obstacle related parameters
 */
struct ObstacleOptions {
    /**
     * Define ObstacleOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(ObstacleOptions);

    // Minimum desired separation from obstacles
    double min_obstacle_dist;

    // buffer zone around obstacles with non-zero penalty costs (should be
    // larger than min_obstacle_dist in order to take effect)
    double inflation_dist;

    // Buffer zone around predicted locations of dynamic obstacles with
    // non-zero penalty costs (should be larger than min_obstacle_dist in order
    // to take effect)
    double dynamic_obstacle_inflation_dist;

    // Specify whether the movement of dynamic obstacles should be predicted by
    // a constant velocity model (this also effects homotopy class planning); If
    // false, all obstacles are considered to be static.
    bool include_dynamic_obstacles;

    // Specify whether the obstacles in the costmap should be taken into account
    // directly
    bool include_costmap_obstacles;

    // Limit the occupied local costmap obstacles taken into account for
    // planning behind the robot (specify distance in meters)
    double costmap_obstacles_behind_robot_dist;

    // The obstacle position is attached to the closest pose on the trajectory
    // to reduce computational effort, but take a number of neighbors into
    // account as well
    int obstacle_poses_affected;

    // If true, the old association strategy is used (for each obstacle, find
    // the nearest TEB pose), otherwise the new one (for each teb pose, find
    // only "relevant" obstacles).
    bool legacy_obstacle_association;
    double obstacle_association_force_inclusion_factor;
    double obstacle_association_cutoff_factor;

    // Define a plugin name of the costmap_converter package (costmap cells are
    // converted to points/lines/polygons)
    std::string costmap_converter_plugin;

    // If true, the costmap converter invokes its callback queue in a different
    // thread
    bool costmap_converter_spin_thread;

    // The rate that defines how often the costmap_converter plugin processes
    // the current costmap (the value should not be much higher than the costmap
    // update rate)
    int costmap_converter_rate;

    // Ratio of the maximum velocities used as an upper bound when reducing the
    // speed due to the proximity to a static obstacles
    double obstacle_proximity_max_velocity_ratio;

    // Distance to a static obstacle for which the velocity should be lower
    double obstacle_proximity_lower_bound;

    // Distance to a static obstacle for which the velocity should be higher
    double obstacle_proximity_upper_bound;
};

/**
 * @brief Optimization related parameters
 */
struct OptimizationOptions {
    /**
     * Define OptimizationOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(OptimizationOptions);

    // Number of solver iterations called in each outerloop iteration
    int inner_iteration_count;

    // Each outerloop iteration automatically resizes the trajectory and invokes
    // the internal optimizer with inner_iteration_count
    int outer_iteration_count;

    // Activate the optimization
    bool optimization_activate;

    // Print verbose information
    bool optimization_verbose;

    // Add a small safety margin to penalty functions for hard-constraint
    // approximations
    double penalty_epsilon;

    // Optimization weight for satisfying the maximum allowed translational
    // velocity
    double weight_max_velocity_x;

    // Optimization weight for satisfying the maximum allowed strafing velocity
    // (in use only for holonomic robots)
    double weight_max_velocity_y;

    // Optimization weight for satisfying the maximum allowed angular velocity
    double weight_max_angular_velocity;

    // Optimization weight for satisfying the maximum allowed translational
    // acceleration
    double weight_max_acceleration_x;

    // Optimization weight for satisfying the maximum allowed strafing
    // acceleration (in use only for holonomic robots)
    double weight_max_acceleration_y;

    // Optimization weight for satisfying the maximum allowed angular
    // acceleration
    double weight_max_angular_acceleration;

    // Optimization weight for satisfying the non-holonomic kinematics
    double weight_kinematics_non_holonomic;

    // Optimization weight for forcing the robot to choose only forward
    // directions (positive transl. velocities, only diffdrive robot)
    double weight_kinematics_forward_drive;

    // Optimization weight for enforcing a minimum turning radius (carlike
    double weight_kinematics_turning_radius;

    // Optimization weight for contracting the trajectory w.r.t. transition time
    double weight_time_optimal;

    // Optimization weight for contracting the trajectory w.r.t. path length
    double weight_shortest_path;

    // Optimization weight for satisfying a minimum separation from obstacles
    double weight_obstacle;

    // Optimization weight for the inflation penalty (should be small)
    double weight_inflation;

    // Optimization weight for satisfying a minimum separation from dynamic
    // obstacles
    double weight_dynamic_obstacle;

    // Optimization weight for the inflation penalty of dynamic obstacles
    // (should be small)
    double weight_dynamic_obstacle_inflation;

    // Optimization weight for satisfying a maximum allowed velocity with
    // respect to the distance to a static obstacle
    double weight_velocity_obstacle_ratio;

    // Optimization weight for minimizing the distance to via-points
    double weight_via_point;

    // Optimization weight for preferring a specific turning direction (->
    // currently only activated if an oscillation is detected, see
    // 'oscillation_recovery')
    double weight_preferred_rotation_direction;

    // Some special weights (currently 'weight_obstacle') are repeatedly scaled
    // by this factor in each outer TEB iteration (weight_new =
    // weight_old*factor); Increasing weights iteratively instead of setting a
    // huge value a-priori leads to better numerical conditions of the
    // underlying optimization problem.
    double weight_adapt_factor;

    // Exponent for nonlinear obstacle cost (cost = linear_cost *
    // obstacle_cost_exponent). Set to 1 to disable nonlinear cost (default)
    double obstacle_cost_exponent;
};

/**
 * @brief Homotopy class planner parameters
 */
struct HomotopyOptions {
    /**
     * Define HomotopyOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(HomotopyOptions);

    // Activate homotopy class planning (Requires much more resources that
    // simple planning, since multiple trajectories are optimized at once).
    bool enable_homotopy_class_planning;

    // Activate multiple threading for planning multiple trajectories in
    // parallel.
    bool enable_multithreading;

    // If true, distinctive trajectories are explored using a simple left-right
    // approach (pass each obstacle on the left or right side) for path
    // generation, otherwise sample possible roadmaps randomly in a specified
    // region between start and goal.
    bool simple_exploration;

    // Specify the maximum number of allowed alternative homotopy classes
    // (limits computational effort)
    int max_number_classes;

    // Specify the maximum number of trajectories to try that are in the same
    // homotopy class as the current trajectory (helps avoid local minima)
    int max_plans_per_homotopy_class;

    // Specify how much trajectory cost must a new candidate have w.r.t. a
    // previously selected trajectory in order to be selected (selection if
    // new_cost < old_cost*factor).
    double selection_cost_hysteresis;

    // Specify a cost reduction in the interval (0,1) for the trajectory in the
    // equivalence class of the initial plan.
    double selection_prefer_initial_plan;

    // Extra scaling of obstacle cost terms just for selecting the 'best'
    // candidate.
    double selection_obstacle_cost_scale;

    // Extra scaling of via-point cost terms just for selecting the 'best'
    // candidate.
    double selection_via_point_cost_scale;

    // If true, time cost is replaced by the total transition time.
    bool selection_alternative_time_cost;

    // At each planning cycle, TEBs other than the current 'best' one will be
    // randomly dropped with this probability. Prevents becoming 'fixated' on
    // sub-optimal alternative homotopies.
    double selection_dropping_probability;

    // Specify a time duration in seconds that needs to be expired before a
    // switch to new equivalence class is allowed
    double switching_blocking_period;

    // Specify the number of samples generated for creating the roadmap graph,
    // if simple_exploration is turend off.
    int roadmap_graph_sample_count;

    // Random keypoints/waypoints are sampled in a rectangular region between
    // start and goal. Specify the width of that region in meters.
    double roadmap_graph_area_width;

    // The length of the rectangular region is determined by the distance
    // between start and goal. This parameter further scales the distance such
    // that the geometric center remains equal!
    double roadmap_graph_area_length_scale;

    // Scale number of obstacle value in order to allow huge number of
    // obstacles. Do not choose it extremly low, otherwise obstacles cannot be
    // distinguished from each other (0.2<H<=1).
    double homotopy_signature_prescaler;

    // Two h-signatures are assumed to be equal, if both the difference of real
    // parts and complex parts are below the specified threshold.
    double homotopy_signature_threshold;

    // If simple_exploration is turned on, this parameter determines the
    // distance on the left and right side of the obstacle at which a new
    // keypoint will be cretead (in addition to min_obstacle_dist).
    double obstacle_keypoint_offset;

    // Specify the value of the normalized scalar product between obstacle
    // heading and goal heading in order to take them (obstacles) into account
    // for exploration [0,1]
    double obstacle_heading_threshold;

    // If true, all trajectories of different topologies are attached to the
    // current set of via-points, otherwise only the trajectory sharing the same
    // one as the initial/global plan.
    bool via_points_all_candidates;

    // Visualize the graph that is created for exploring new homotopy classes.
    bool visualize_homotopy_graph;

    // If this value is bigger than 0, the trajectory and obstacles are
    // visualized in 3d using the time as the z-axis scaled by this value.
    // Most useful for dynamic obstacles.
    double visualize_with_time_as_z_axis_scale;

    // If enabled, the planner will discard the plans detouring backwards with
    // respect to the best plan
    bool delete_detours_backwards;

    // A plan is considered a detour if its start orientation differs more than
    // this from the best plan
    double detours_orientation_tolerance;

    // Length of the vector used to compute the start orientation of a plan
    double length_start_orientation_vector;

    // Detours are discarted if their execution time / the execution time of the
    // best teb is > this
    double max_ratio_detours_duration_best_duration;
};

/**
 * @brief Recovery related parameters
 */
struct RecoveryOptions {
    /**
     * Define RecoveryOptions::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(RecoveryOptions);

    // Allows the planner to shrink the horizon temporary (50%) in case of
    // automatically detected issues.
    bool shrink_horizon_backup;

    // Specify minimum duration for the reduced horizon in case an infeasible
    // trajectory is detected.
    double shrink_horizon_min_duration;

    // Try to detect and resolve oscillations between multiple solutions in the
    // same equivalence class (robot frequently switches between
    // left/right/forward/backwards)
    bool oscillation_recovery;

    // Threshold for the average normalized linear velocity: if
    // oscillation_v_eps and oscillation_omega_eps are not exceeded both, a
    // possible oscillation is detected
    double oscillation_v_eps;

    // Threshold for the average normalized angular velocity: if
    // oscillation_v_eps and oscillation_omega_eps are not exceeded both, a
    // possible oscillation is detected
    double oscillation_omega_eps;

    // Minumum duration [sec] for which the recovery mode is activated after an
    // oscillation is detected.
    double oscillation_recovery_min_duration;

    // Filter length/duration [sec] for the detection of oscillations
    double oscillation_filter_duration;

    // True to enable divergence detection.
    bool divergence_detection_enable;

    // Maximum acceptable Mahalanobis distance above which it is assumed that
    // the optimization diverged.
    int divergence_detection_max_chi_squared;
};

/**
 * @brief TEB planner configuration
 */
struct TimedElasticBandConfig {
    /**
     * Define TimedElasticBandConfig::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(TimedElasticBandConfig);

    // Topic name of the odometry message, provided by the robot driver or
    // simulator
    std::string odom_topic;

    // Global planning frame
    std::string map_frame;

    RobotFootprintPtr robot_footprint;
    std::string model_name;
    double radius;
    std::vector<double> line_start, line_end;
    double front_offset, front_radius, rear_offset, rear_radius;
    std::string footprint_string;

    TrajectoryOptions trajectory;
    RobotOptions robot;
    GoalToleranceOptions goal_tolerance;
    ObstacleOptions obstacles;
    OptimizationOptions optimization;
    HomotopyOptions homotopy;
    RecoveryOptions recovery;

    /**
     * @brief Constructor for TimedElasticBandConfig
     */
    TimedElasticBandConfig();
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
