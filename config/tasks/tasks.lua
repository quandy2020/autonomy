-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Loaded by tasks::CreateOptions(config_dir, "tasks/tasks.lua") from
-- AutonomyNode::Configure() and standalone tooling.
--
-- BT plugins: set AUTONOMY_BT_PLUGIN_PATH (colon-separated dirs) or leave
-- plugin_lib_path empty to use the build/install lib directory.
-- BT XML: resolved under <config>/tasks/behavior_tree/ (see bt_utils.cpp).

if not AUTONOMY_COMMON then
    include "common.lua"
end

-- Plugins used by navigate_to_pose.xml and navigate_through_poses.xml.
local plugin_lib_names_navigation = {
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
    "autonomy_behavior_tree_action_clear_costmap_service",
    "autonomy_behavior_tree_condition_goal_reached_condition",
    "autonomy_behavior_tree_condition_initial_pose_received_condition",
    "autonomy_behavior_tree_condition_is_path_valid_condition",
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

    -- BT tick period (ms); BehaviorTreeEngine sleep between ticks.
    bt_loop_duration = 10,
    -- Reserved for odom filtering (not wired yet).
    filter_duration = 0.0,
    -- Service / action wait timeout in milliseconds (blackboard server_timeout).
    default_server_timeout = 20000,
    -- Local survival mode cap (seconds); see navigate_to_pose.xml TimeExpired.
    local_survival_timeout = 120.0,

    plugin_lib_names = plugin_lib_names_navigation,
    plugin_lib_path = "",

    -- Informational; BtNavigator registers navigate_to_pose / navigate_through_poses.
    navigators = {
        "navigate_to_pose",
        "navigate_through_poses",
    },

    navigate_to_pose = {
        enable = true,
        default_behavior_tree_file = "navigate_to_pose.xml",
        goal_blackboard_key = "goal",
        path_blackboard_key = "path",
    },

    navigate_through_poses = {
        enable = true,
        default_behavior_tree_file = "navigate_through_poses.xml",
        goals_blackboard_key = "goals",
        path_blackboard_key = "path",
    },

    -- Phase 3: autolink SimpleActionServer (navigate_to_pose / navigate_through_poses).
    enable_autolink_action_servers = true,
}

return { tasks = tasks }
