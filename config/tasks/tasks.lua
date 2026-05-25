-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- tasks::Task loads this file in attachScheduler (see autonomy/tasks/options.cpp).
-- BT plugin search: set AUTONOMY_BT_PLUGIN_PATH or leave plugin_lib_path empty
-- to use the build/install lib directory.

include "common.lua"

-- Plugins required by config/tasks/behavior_tree/navigate_to_pose.xml
local plugin_lib_names_minimal = {
    "autonomy_behavior_tree_action_back_up_action",
    "autonomy_behavior_tree_action_compute_path_to_pose_action",
    "autonomy_behavior_tree_action_compute_path_through_poses_action",
    "autonomy_behavior_tree_action_controller_selector_node",
    "autonomy_behavior_tree_action_drive_on_heading_action",
    "autonomy_behavior_tree_action_follow_path_action",
    "autonomy_behavior_tree_action_planner_selector_node",
    "autonomy_behavior_tree_action_smooth_path_action",
    "autonomy_behavior_tree_action_smoother_selector_node",
    "autonomy_behavior_tree_action_spin_action",
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

    plugin_lib_names = plugin_lib_names_minimal,
    -- Empty: use AUTONOMY_BT_PLUGIN_PATH env or build lib directory.
    plugin_lib_path = "",

    -- Registered by BtNavigator::setupNavigators (see navigator/bt_navigator.cpp).
    navigators = {
        "navigate_to_pose",
        "navigate_through_poses",
        "explore_to_anywhere",
    },

    navigate_to_pose = {
        enable = true,
        default_behavior_tree_file = "navigate_to_pose.xml",
    },

    navigate_through_poses = {
        enable = true,
        default_behavior_tree_file = "navigate_through_poses.xml",
    },
    explore_to_anywhere = {
        enable = true,
        default_behavior_tree_file = "explore_to_anywhere.xml",
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

return { tasks = tasks }
