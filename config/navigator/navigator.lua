-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Navigator / BT options → task::navigation::LoadOptions (see task/navigation/proto/navigator_options.proto).
-- Loaded from autonomy.lua or navigator/navigator.lua via system::Autonomy::Configure().
--
-- BT plugins: AUTONOMY_BT_PLUGIN_PATH, or leave plugin_lib_path empty for install lib.
-- BT XML: behavior_tree/*.xml (see behavior_tree/README.md).

if AUTONOMY_COMMON == nil then
    include "common.lua"
end

navigator = {
    -- Frames and navigation defaults (single source: common.lua).
    global_frame = AUTONOMY_COMMON.global_frame,
    robot_base_frame = AUTONOMY_COMMON.robot_base_frame,
    default_planner_id = AUTONOMY_COMMON.default_planner_id,
    default_controller_id = AUTONOMY_COMMON.default_controller_id,
    default_goal_checker_id = AUTONOMY_COMMON.default_goal_checker_id,
    default_smoother_id = AUTONOMY_COMMON.default_smoother_id,
    goal_reached_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,

    -- Behavior tree tick / timeout (ms unless noted).
    bt_loop_duration = 10,
    default_server_timeout = 20000,
    local_survival_timeout = 120.0,  -- seconds; navigate_to_pose local survival

    plugin_lib_path = "",
    -- Must match autonomy/task/navigation/plugins/*.cpp and cmake plugin targets.
    plugin_lib_names = {
        -- action (one .so per file; clear_costmap_service registers 3 nodes like nav2)
        "autonomy_behavior_tree_action_back_up_action",
        "autonomy_behavior_tree_action_back_up_cancel_node",
        "autonomy_behavior_tree_action_clear_costmap_service",
        "autonomy_behavior_tree_action_compute_path_through_poses_action",
        "autonomy_behavior_tree_action_compute_path_to_pose_action",
        "autonomy_behavior_tree_action_controller_cancel_node",
        "autonomy_behavior_tree_action_controller_selector_node",
        "autonomy_behavior_tree_action_drive_on_heading_action",
        "autonomy_behavior_tree_action_drive_on_heading_cancel_node",
        "autonomy_behavior_tree_action_follow_path_action",
        "autonomy_behavior_tree_action_get_pose_from_path_action",
        "autonomy_behavior_tree_action_goal_checker_selector_node",
        "autonomy_behavior_tree_action_planner_selector_node",
        "autonomy_behavior_tree_action_progress_checker_selector_node",
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
        -- condition (one .so per file, names match plugins/condition/*.cpp)
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
        "autonomy_behavior_tree_condition_would_a_smoother_recovery_help_condition",
        -- control
        "autonomy_behavior_tree_control_pipeline_sequence",
        "autonomy_behavior_tree_control_recovery_node",
        "autonomy_behavior_tree_control_round_robin_node",
        -- decorator (one .so per file, names match plugins/decorator/*.cpp)
        "autonomy_behavior_tree_decorator_distance_controller",
        "autonomy_behavior_tree_decorator_goal_updated_controller",
        "autonomy_behavior_tree_decorator_goal_updater_node",
        "autonomy_behavior_tree_decorator_path_longer_on_approach",
        "autonomy_behavior_tree_decorator_rate_controller",
        "autonomy_behavior_tree_decorator_single_trigger_node",
        "autonomy_behavior_tree_decorator_speed_controller",
    },

    navigate_to_pose = {
        enable = true,
        behavior_tree_file = "navigate_to_pose.xml",
    },

    navigate_through_poses = {
        enable = true,
        behavior_tree_file = "navigate_through_poses.xml",
    },

    enable_autolink_action_servers = true,
}

return { navigator = navigator }
