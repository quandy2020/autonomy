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

-- Set plugin_index_path / plugin_lib_path when env is not set (avoids "plugin of class ... have not been loaded").
-- Example (Docker/install): "/workspace/autonomy/install/autonomy/share/autolink_plugin_index"
-- Example (lib):            "/workspace/autonomy/install/autonomy/lib"
tasks = {
    global_frame = "map",
    robot_base_frame = "base_link",
    odom_topic = "odom",
    bt_loop_duration = 10,
    filter_duration = 0.3,
    default_server_timeout = 20,
    wait_for_service_timeout = 1000,
    service_introspection_mode = "disabled",
    local_survival_timeout = 120.0,
    plugin_lib_names = {
        "autonomy_behavior_tree_action_append_goal_pose_to_goals_action",
        "autonomy_behavior_tree_action_assisted_teleop_action",
        "autonomy_behavior_tree_action_assisted_teleop_cancel_node",
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
    },
    plugin_index_path = "/workspace/autonomy/install/autonomy/share/autolink_plugin_index",
    -- Directory containing libautonomy_behavior_tree_*.so (built by CMake).
    plugin_lib_path = "/workspace/autonomy/src/autonomy/build/lib",
    navigators = {
        "navigate_to_pose",
        "navigate_through_poses",
        "navigate_to_docking",
        "track_to_target",
        "explore_to_anywhere",
    },
    -- navigate_to_pose = {
    --     enable = false,
    --     plugin = "autonomy_tasks_navigator_navigate_to_pose",
    --     default_behavior_tree_file = "navigate_to_pose_w_replanning_and_recovery.xml",
    -- },
    -- navigate_through_poses = {
    --     enable = false,
    --     plugin = "autonomy_tasks_navigator_navigate_through_poses",
    --     default_behavior_tree_file = "navigate_through_poses_w_replanning_and_recovery.xml",
    -- },
    -- navigate_to_docking = {
    --     enable = false,
    --     plugin = "autonomy_tasks_navigator_navigate_to_docking",
    --     default_behavior_tree_file = "navigate_to_dock.xml",
    -- },
    track_to_target = {
        enable = true,
        plugin = "autonomy_tasks_navigator_track_to_target",
        default_behavior_tree_file = "track_to_target.xml",
    },
    -- explore_to_anywhere = {
    --     enable = false,
    --     plugin = "autonomy_tasks_navigator_explore_to_anywhere",
    --     default_behavior_tree_file = "explore_to_anywhere.xml",
    -- },
    error_code_name_prefixes = {
        "assisted_teleop", "backup", "compute_path", "dock_robot", "drive_on_heading",
        "follow_path", "nav_thru_poses", "nav_to_pose", "route", "spin", "smoother",
        "undock_robot", "wait",
    },
    enable_groot_monitoring = false,
    groot_server_port = 1667,
}

-- LuaParameterDictionary expects the script to return a table (root must be a table).
-- LoadOptions() expects the root to have key "tasks".
return { tasks = tasks }