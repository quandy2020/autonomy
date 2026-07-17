-- Copyright 2026 The Openbot Authors
--
-- Per-task behavior tree profiles for autonomy/task/apps.
-- Paths are relative to config_directory (default: config/).
-- BT nodes are linked into libautonomy.so (no separate plugin_lib_names).

task = {
  config_directory = "config",

  behavior_trees = {
    navigation = {
      default_tree_file = "task/behavior_tree/navigation/navigate_to_pose.xml",
      alternate_tree_file = "task/behavior_tree/navigation/navigate_through_poses.xml",
      bt_loop_duration_ms = 10,
      default_server_timeout_ms = 20000,
    },
    tracking = {
      default_tree_file = "task/behavior_tree/tracking/follow_target.xml",
      alternate_tree_file = "task/behavior_tree/tracking/follow_person.xml",
      bt_loop_duration_ms = 20,
      default_server_timeout_ms = 20000,
    },
    teleop = {
      default_tree_file = "task/behavior_tree/teleop/teleop.xml",
      bt_loop_duration_ms = 50,
      default_server_timeout_ms = 20000,
    },
    exploration = {
      default_tree_file = "task/behavior_tree/exploration/explore.xml",
      bt_loop_duration_ms = 10,
      default_server_timeout_ms = 20000,
    },
    charging = {
      default_tree_file = "task/behavior_tree/charging/dock.xml",
      bt_loop_duration_ms = 10,
      default_server_timeout_ms = 20000,
    },
    mapping = {
      default_tree_file = "task/behavior_tree/mapping/map_load.xml",
      alternate_tree_file = "task/behavior_tree/mapping/map_set_pose.xml",
      bt_loop_duration_ms = 100,
      default_server_timeout_ms = 20000,
    },
    localization = {
      default_tree_file = "task/behavior_tree/localization/localization.xml",
      bt_loop_duration_ms = 100,
      default_server_timeout_ms = 20000,
    },
  },
}

return task
