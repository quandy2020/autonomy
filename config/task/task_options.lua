-- Copyright 2026 The Openbot Authors
--
-- Per-task behavior tree profiles for autonomy/task/apps.
-- Paths are relative to config_directory (default: config/).
-- plugin_lib_names must match CMake targets from task_behavior_tree_plugins.cmake.

task = {
  config_directory = "config",

  behavior_trees = {
    navigation = {
      default_tree_file = "task/behavior_tree/navigation/navigate_to_pose.xml",
      alternate_tree_file = "task/behavior_tree/navigation/navigate_through_poses.xml",
      bt_loop_duration_ms = 10,
      default_server_timeout_ms = 20000,
      plugin_lib_names = {
        "autonomy_task_navigation_action_compute_path_action",
        "autonomy_task_navigation_action_smooth_path_action",
        "autonomy_task_navigation_action_follow_path_action",
        "autonomy_task_navigation_action_motion_actions",
        "autonomy_task_navigation_action_costmap_actions",
        "autonomy_task_navigation_condition_goal_reached_condition",
        "autonomy_task_navigation_condition_servers_ready_condition",
        "autonomy_task_navigation_condition_path_valid_condition",
      },
    },
    tracking = {
      default_tree_file = "task/behavior_tree/tracking/follow_target.xml",
      alternate_tree_file = "task/behavior_tree/tracking/follow_person.xml",
      bt_loop_duration_ms = 20,
      plugin_lib_names = {
        "autonomy_task_tracking_action_compute_follow_goal_action",
        "autonomy_task_tracking_condition_target_locked_condition",
        "autonomy_task_navigation_action_compute_path_action",
        "autonomy_task_navigation_action_smooth_path_action",
        "autonomy_task_navigation_action_follow_path_action",
        "autonomy_task_navigation_condition_servers_ready_condition",
        "autonomy_task_navigation_condition_path_valid_condition",
        "autonomy_task_navigation_condition_goal_reached_condition",
      },
    },
    teleop = {
      default_tree_file = "task/behavior_tree/teleop/teleop.xml",
      bt_loop_duration_ms = 50,
      plugin_lib_names = {
        "autonomy_task_teleop_action_apply_teleop_velocity_action",
        "autonomy_task_teleop_condition_teleop_watchdog_ok_condition",
      },
    },
    exploration = {
      default_tree_file = "task/behavior_tree/exploration/explore.xml",
      bt_loop_duration_ms = 10,
      plugin_lib_names = {
        "autonomy_task_exploration_action_select_frontier_action",
        "autonomy_task_exploration_action_save_exploration_map_action",
        "autonomy_task_exploration_condition_frontier_available_condition",
        "autonomy_task_navigation_action_compute_path_action",
        "autonomy_task_navigation_action_smooth_path_action",
        "autonomy_task_navigation_action_follow_path_action",
        "autonomy_task_navigation_condition_servers_ready_condition",
        "autonomy_task_navigation_condition_path_valid_condition",
        "autonomy_task_navigation_condition_goal_reached_condition",
      },
    },
    charging = {
      default_tree_file = "task/behavior_tree/charging/dock.xml",
      bt_loop_duration_ms = 10,
      plugin_lib_names = {
        "autonomy_task_charging_action_dock_search_action",
        "autonomy_task_charging_action_dock_connect_action",
        "autonomy_task_navigation_action_compute_path_action",
        "autonomy_task_navigation_action_smooth_path_action",
        "autonomy_task_navigation_action_follow_path_action",
        "autonomy_task_navigation_condition_servers_ready_condition",
        "autonomy_task_navigation_condition_path_valid_condition",
      },
    },
    mapping = {
      default_tree_file = "task/behavior_tree/mapping/map_load.xml",
      alternate_tree_file = "task/behavior_tree/mapping/map_set_pose.xml",
      bt_loop_duration_ms = 100,
      plugin_lib_names = {
        "autonomy_task_mapping_action_load_map_action",
        "autonomy_task_mapping_action_set_initial_pose_action",
        "autonomy_task_mapping_action_clear_costmap_action",
        "autonomy_task_navigation_action_costmap_actions",
      },
    },
    localization = {
      default_tree_file = "task/behavior_tree/localization/localization.xml",
      bt_loop_duration_ms = 100,
      plugin_lib_names = {
        "autonomy_task_localization_action_start_localization_action",
        "autonomy_task_localization_action_stop_localization_action",
      },
    },
  },
}

return task
