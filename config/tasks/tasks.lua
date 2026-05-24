-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- TaskScheduler loads this file directly (see autonomy/tasks/options.cpp).
-- BT plugin search: set AUTONOMY_BT_PLUGIN_PATH or leave plugin_lib_path empty
-- to use the build/install lib directory.

include "common.lua"

-- Plugins required by config/tasks/behavior_tree/navigate_to_pose.xml
local plugin_lib_names_minimal = {
    "autonomy_behavior_tree_action_assisted_teleop_velocity_action",
    "autonomy_behavior_tree_action_back_up_action",
    "autonomy_behavior_tree_action_clear_costmap_service",
    "autonomy_behavior_tree_action_compute_path_to_pose_action",
    "autonomy_behavior_tree_action_compute_path_through_poses_action",
    "autonomy_behavior_tree_action_controller_selector_node",
    "autonomy_behavior_tree_action_drive_on_heading_action",
    "autonomy_behavior_tree_action_follow_path_action",
    "autonomy_behavior_tree_action_planner_selector_node",
    "autonomy_behavior_tree_action_smooth_path_action",
    "autonomy_behavior_tree_action_smoother_selector_node",
    "autonomy_behavior_tree_action_reinitialize_global_localization_service",
    "autonomy_behavior_tree_action_spin_action",
    "autonomy_behavior_tree_action_teleop_drive_action",
    "autonomy_behavior_tree_action_wait_action",
    "autonomy_behavior_tree_condition_goal_reached_condition",
    "autonomy_behavior_tree_condition_initial_pose_received_condition",
    "autonomy_behavior_tree_condition_is_path_valid_condition",
    "autonomy_behavior_tree_condition_is_teleop_linear_sign_condition",
    "autonomy_behavior_tree_condition_is_teleop_mode_condition",
    "autonomy_behavior_tree_condition_is_teleop_rotate_requested_condition",
    "autonomy_behavior_tree_condition_time_expired_condition",
    "autonomy_behavior_tree_condition_transform_available_condition",
    "autonomy_behavior_tree_control_pipeline_sequence",
    "autonomy_behavior_tree_control_recovery_node",
    "autonomy_behavior_tree_control_round_robin_node",
    "autonomy_behavior_tree_decorator_rate_controller",
}

-- Full Nav2-style plugin set (other behavior trees, route/smoother/battery nodes).
local plugin_lib_names_full = {
    "autonomy_behavior_tree_action_append_goal_pose_to_goals_action",
    "autonomy_behavior_tree_action_assisted_teleop_action",
    "autonomy_behavior_tree_action_assisted_teleop_cancel_node",
    "autonomy_behavior_tree_action_assisted_teleop_velocity_action",
    "autonomy_behavior_tree_action_back_up_action",
    "autonomy_behavior_tree_action_back_up_cancel_node",
    "autonomy_behavior_tree_action_clear_costmap_service",
    "autonomy_behavior_tree_action_compute_and_track_route_action",
    "autonomy_behavior_tree_action_compute_and_track_route_cancel_node",
    "autonomy_behavior_tree_action_compute_path_through_poses_action",
    "autonomy_behavior_tree_action_compute_path_to_pose_action",
    "autonomy_behavior_tree_action_compute_route_action",
    "autonomy_behavior_tree_action_concatenate_paths_action",
    "autonomy_behavior_tree_action_controller_cancel_node",
    "autonomy_behavior_tree_action_controller_selector_node",
    "autonomy_behavior_tree_action_drive_on_heading_action",
    "autonomy_behavior_tree_action_drive_on_heading_cancel_node",
    "autonomy_behavior_tree_action_extract_route_nodes_as_goals_action",
    "autonomy_behavior_tree_action_follow_path_action",
    "autonomy_behavior_tree_action_get_current_pose_action",
    "autonomy_behavior_tree_action_get_next_few_goals_action",
    "autonomy_behavior_tree_action_get_pose_from_path_action",
    "autonomy_behavior_tree_action_goal_checker_selector_node",
    "autonomy_behavior_tree_action_navigate_through_poses_action",
    "autonomy_behavior_tree_action_navigate_to_pose_action",
    "autonomy_behavior_tree_action_planner_selector_node",
    "autonomy_behavior_tree_action_progress_checker_selector_node",
    "autonomy_behavior_tree_action_reinitialize_global_localization_service",
    "autonomy_behavior_tree_action_remove_in_collision_goals_action",
    "autonomy_behavior_tree_action_remove_passed_goals_action",
    "autonomy_behavior_tree_action_smooth_path_action",
    "autonomy_behavior_tree_action_smoother_selector_node",
    "autonomy_behavior_tree_action_spin_action",
    "autonomy_behavior_tree_action_spin_cancel_node",
    "autonomy_behavior_tree_action_teleop_drive_action",
    "autonomy_behavior_tree_action_truncate_path_action",
    "autonomy_behavior_tree_action_truncate_path_local_action",
    "autonomy_behavior_tree_action_wait_action",
    "autonomy_behavior_tree_action_wait_cancel_node",
    "autonomy_behavior_tree_condition_are_error_codes_present_condition",
    "autonomy_behavior_tree_condition_are_poses_near_condition",
    "autonomy_behavior_tree_condition_distance_traveled_condition",
    "autonomy_behavior_tree_condition_globally_updated_goal_condition",
    "autonomy_behavior_tree_condition_goal_reached_condition",
    "autonomy_behavior_tree_condition_goal_updated_condition",
    "autonomy_behavior_tree_condition_initial_pose_received_condition",
    "autonomy_behavior_tree_condition_is_battery_charging_condition",
    "autonomy_behavior_tree_condition_is_battery_low_condition",
    "autonomy_behavior_tree_condition_is_path_valid_condition",
    "autonomy_behavior_tree_condition_is_stopped_condition",
    "autonomy_behavior_tree_condition_is_stuck_condition",
    "autonomy_behavior_tree_condition_is_teleop_linear_sign_condition",
    "autonomy_behavior_tree_condition_is_teleop_mode_condition",
    "autonomy_behavior_tree_condition_is_teleop_rotate_requested_condition",
    "autonomy_behavior_tree_condition_path_expiring_timer_condition",
    "autonomy_behavior_tree_condition_time_expired_condition",
    "autonomy_behavior_tree_condition_transform_available_condition",
    "autonomy_behavior_tree_condition_would_a_controller_recovery_help_condition",
    "autonomy_behavior_tree_condition_would_a_planner_recovery_help_condition",
    "autonomy_behavior_tree_condition_would_a_route_recovery_help_condition",
    "autonomy_behavior_tree_condition_would_a_smoother_recovery_help_condition",
    "autonomy_behavior_tree_control_nonblocking_sequence",
    "autonomy_behavior_tree_control_persistent_sequence",
    "autonomy_behavior_tree_control_pipeline_sequence",
    "autonomy_behavior_tree_control_recovery_node",
    "autonomy_behavior_tree_control_round_robin_node",
    "autonomy_behavior_tree_decorator_distance_controller",
    "autonomy_behavior_tree_decorator_goal_updated_controller",
    "autonomy_behavior_tree_decorator_goal_updater_node",
    "autonomy_behavior_tree_decorator_path_longer_on_approach",
    "autonomy_behavior_tree_decorator_rate_controller",
    "autonomy_behavior_tree_decorator_single_trigger_node",
    "autonomy_behavior_tree_decorator_speed_controller",
}

-- "minimal" | "full"
-- When built with -DAUTONOMY_BT_PLUGINS_BUNDLED=ON, set plugin_profile unused and use:
--   plugin_lib_names = { "autonomy_behavior_tree_plugins" }
local plugin_profile = "minimal"

tasks = {
    global_frame = AUTONOMY_COMMON.global_frame,
    robot_base_frame = AUTONOMY_COMMON.robot_base_frame,
    odom_topic = AUTONOMY_COMMON.odom_topic,

    default_planner_id = AUTONOMY_COMMON.default_planner_id,
    default_controller_id = AUTONOMY_COMMON.default_controller_id,
    default_goal_checker_id = AUTONOMY_COMMON.default_goal_checker_id,
    goal_reached_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,

    -- BT main loop period (ms). FollowPath runs once per tick; RateController limits replanning.
    bt_loop_duration = 10,
    -- 0 until odom subscription is wired; demo uses SeedZeroOdometry in main.
    filter_duration = 0.0,
    default_server_timeout = 20,
    local_survival_timeout = 120.0,

    plugin_profile = plugin_profile,
    plugin_lib_names_minimal = plugin_lib_names_minimal,
    plugin_lib_names_full = plugin_lib_names_full,
    -- Empty: resolved from profile below; override to pin an explicit list.
    plugin_lib_names = {},
    -- Empty: use AUTONOMY_BT_PLUGIN_PATH env or build lib directory.
    plugin_lib_path = "",

    -- Registered by NavigatorFactory (see autonomy/tasks/navigator/navigator_factory.cpp).
    navigators = {
        "navigate_to_pose",
        "navigate_through_poses",
        "navigate_to_docking",
        "track_to_target",
        "explore_to_anywhere",
        "teleop_drive",
    },

    navigate_to_pose = {
        enable = true,
        default_behavior_tree_file = "navigate_to_pose.xml",
    },

    navigate_through_poses = {
        enable = true,
        default_behavior_tree_file = "navigate_through_poses.xml",
    },
    navigate_to_docking = {
        enable = true,
        default_behavior_tree_file = "navigate_to_dock.xml",
    },
    track_to_target = {
        enable = true,
        default_behavior_tree_file = "track_to_target.xml",
    },
    explore_to_anywhere = {
        enable = true,
        default_behavior_tree_file = "explore_to_anywhere.xml",
    },

    teleop_drive = {
        enable = true,
        default_behavior_tree_file = "teleop_drive.xml",
    },

    teleop_drive_options = {
        default_max_linear_vel = 0.5,
        default_max_angular_vel = 1.5,
        cmd_stale_timeout_sec = 0.5,
        projection_time_sec = 1.5,
        simulation_step_sec = 0.1,
    },

    error_code_name_prefixes = {
        "assisted_teleop", "backup", "compute_path", "dock_robot", "drive_on_heading",
        "follow_path", "nav_thru_poses", "nav_to_pose", "route", "spin", "smoother",
        "undock_robot", "wait",
    },
    enable_groot_monitoring = false,
    groot_server_port = 1667,
}

if plugin_profile == "full" then
    tasks.plugin_lib_names = plugin_lib_names_full
else
    tasks.plugin_lib_names = plugin_lib_names_minimal
end

return { tasks = tasks }
