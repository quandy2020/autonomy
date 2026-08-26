-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.
if AUTONOMY_COMMON == nil then
  include "common.lua"
end

AUTONOMY_CONTROLLER = {
    -- Must match 1/model_dt (0.05 -> 20 Hz) for MPPI control-sequence shifting.
    controller_frequency = 20.0,
    -- Allow short TF/control hiccups without tearing down FollowPath.
    -- Allow long MPPI laps in controller_test before FollowPath aborts.
    failure_tolerance = 30.0,
    publish_zero_velocity = true,

    controller_plugins = {
        "graceful_controller:GracefulController",
        "mppi_controller:MppiController",
    },
    controller_plugin_libraries = {},

    -- Shared with navigator.lua / BT GoalReached (see config/common.lua).
    goal_checker = {
        xy_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
        -- Keep heading convergence stricter than XY tolerance at goal.
        yaw_goal_tolerance = 0.20,
        stateful = true,
        -- Remaining path length must be within this before XY/yaw checks.
        path_length_tolerance = 0.35,
    },
    progress_checker = {
        -- Allow in-place rotation / slow start before declaring stuck.
        required_movement_radius = 0.15,
        movement_time_allowance = 30.0,
    },

    costmap = {
        enabled = true,
        name = "local_costmap",
        frame_id = AUTONOMY_COMMON.global_frame,
        resolution = 0.05,
        width = 10.0,
        height = 10.0,
        robot_radius = 0.22,
        -- Rectangle footprint (x: forward, y: left), counter-clockwise.
        footprint = {
            {x = 0.18, y = 0.14},
            {x = 0.18, y = -0.14},
            {x = -0.18, y = -0.14},
            {x = -0.18, y = 0.14},
        },
        update_frequency = 10.0,
        rolling_window = true,
        publish_frequency = 5.0,
        always_send_full_costmap = true,
        plugins = {"static_layer", "obstacle_layer", "inflation_layer"},

        static_layer = {
            plugin = "libautonomy_map_layers_static_layer.so",
            enabled = true,
            subscribe_to_updates = false,
            transform_tolerance = 0.1,
            footprint_clearing_enabled = false,
            map_topic = "/map",
            track_unknown_space = true,
            lethal_cost_threshold = 65,
            trinary_costmap = true,
        },

        obstacle_layer = {
            plugin = "libautonomy_map_layers_obstacle_layer.so",
            enabled = true,
            footprint_clearing_enabled = true,
            sensor_sources = {
                scan = {
                    topic = "/scan",
                    data_type = "LaserScan",
                    marking = true,
                    clearing = true,
                    obstacle_max_range = 5.0,
                    raytrace_max_range = 5.0,
                },
            },
        },

        inflation_layer = {
            plugin = "libautonomy_map_layers_inflation_layer.so",
            enabled = true,
            cost_scaling_factor = 3.0,
            inflation_radius = 0.40,
            inflate_unknown = false,
            inflate_around_unknown = false,
        },
    },

    graceful_controller = {
        transform_tolerance = AUTONOMY_COMMON.transform_tolerance,
        max_lookahead = 1.0,
        min_lookahead = 0.25,
        max_robot_pose_search_dist = 8.0,
        k_phi = 2.0,
        k_delta = 1.0,
        beta = 0.4,
        lambda = 2.0,
        v_linear_max = 0.5,
        v_linear_min = 0.1,
        v_angular_max = 1.0,
        v_angular_min_in_place = 0.25,
        slowdown_radius = 1.5,
        deceleration_max = 2.5,
        initial_rotation = true,
        initial_rotation_tolerance = 0.75,
        prefer_final_rotation = true,
        rotation_scaling_factor = 0.5,
        allow_backward = false,
        use_collision_detection = false,
        in_place_collision_resolution = 0.1,
        footprint_scaling_linear_vel = 0.5,
        footprint_scaling_factor = 0.25,
        footprint_scaling_step = 0.1,
        final_rotation_search_step = 0.1,
        obstacle_cost_margin = 1,
    },

    mppi_controller = {
        -- Progress checker plugin
        progress_checker = {
            plugin = "nav2_controller::SimpleProgressChecker",
            required_movement_radius = 0.5,
            movement_time_allowance = 10.0,
        },
        -- Goal checker plugin (legacy path; prefer top-level goal_checker above)
        goal_checker = {
            plugin = "nav2_controller::SimpleGoalChecker",
            xy_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
            yaw_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
            stateful = true,
        },
        name = "mppi_controller",
        plugin = "nav2_mppi_controller::MPPIController",
        time_steps = 48,
        model_dt = 0.05,
        batch_size = 2000,
        vx_std = 0.2,
        vy_std = 0.2,
        wz_std = 0.4,
        vx_max = 0.5,
        vx_min = -0.35,
        vy_max = 0.5,
        wz_max = 1.9,
        iteration_count = 1,
        temperature = 0.3,
        gamma = 0.015,
        motion_model = "DiffDrive",
        visualize = false,
        ax_max = 3.0,
        ax_min = -3.0,
        ay_max = 3.0,
        ay_min = -3.0,
        az_max = 3.5,
        retry_attempt_limit = 3,
        regenerate_noises = false,
        open_loop = false,
        clamp_raw_controls = false,
        model_delay_vx = 0.0,
        model_delay_vy = 0.0,
        model_delay_wz = 0.0,
        sgf_order = 2,
        PathHandler = {
            max_robot_pose_search_dist = 0.0,
            prune_distance = 1.5,
            transform_tolerance = 0.1,
            enforce_path_inversion = false,
        },
        TrajectoryVisualizer = {
            trajectory_step = 8,
            time_step = 4,
        },
        AckermannConstraints = {
            min_turning_r = 0.2,
        },
        critics = {
            "ConstraintCritic", 
            "CostCritic", 
            "GoalCritic", 
            "GoalAngleCritic",
            "PathAlignCritic", 
            "PathFollowCritic", 
            "PathAngleCritic", 
            "PreferForwardCritic",
        },
        ConstraintCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 4.0,
        },
        GoalCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 5.0,
            threshold_to_consider = 1.4,
        },
        GoalAngleCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 3.0,
            threshold_to_consider = 0.5,
        },
        PreferForwardCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 5.0,
            threshold_to_consider = 0.5,
        },
        CostCritic = {
            enabled = true,
            cost_power = 1,
            -- Stronger avoidance vs PathAlign/PathFollow so detours win.
            cost_weight = 10.0,
            critical_cost = 300.0,
            -- Circular robot: center-point + inscribed cost as collision.
            consider_footprint = false,
            collision_cost = 1000000.0,
            near_goal_distance = 1.0,
            trajectory_point_step = 3,
        },
        PathAlignCritic = {
            enabled = true,
            cost_power = 1,
            -- Lower than default so CostCritic can deviate around obstacles.
            cost_weight = 8.0,
            max_path_occupancy_ratio = 0.05,
            trajectory_point_step = 6,
            threshold_to_consider = 0.5,
            offset_from_furthest = 20,
            use_path_orientations = false,
        },
        PathFollowCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 5.0,
            offset_from_furthest = 5,
            threshold_to_consider = 1.4,
        },
        PathAngleCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 2.0,
            offset_from_furthest = 4,
            threshold_to_consider = 0.5,
            max_angle_to_furthest = 1.0,
            forward_preference = true,
        },
        ObstaclesCritic = {
            enabled = false, 
            cost_power = 1, 
            repulsion_weight = 1.5, 
            critical_weight = 20.0,
            consider_footprint = false, 
            collision_cost = 10000.0,
            collision_margin_distance = 0.1, 
            near_goal_distance = 0.5,
        },
        VelocityDeadbandCritic = {
            enabled = false, 
            cost_power = 1, 
            cost_weight = 35.0,
            deadband_velocities = {
                0.05, 
                0.05, 
                0.05
            },
        },
        TwirlingCritic = {
            enabled = false, 
            twirling_cost_power = 1, 
            twirling_cost_weight = 10.0,
        },
    },

}

-- LuaParameterDictionary expects the script to return a table when loaded directly.
return { AUTONOMY_CONTROLLER = AUTONOMY_CONTROLLER }