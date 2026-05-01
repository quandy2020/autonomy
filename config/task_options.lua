-- Copyright 2025 The Openbot Authors (duyongquan)
-- Task options for autonomy.tasks.launcher (required top-level key: "tasks")
-- Licensed under the Apache License, Version 2.0.
--
-- Required for navigator plugins to load (avoid "plugin of class ... have not been loaded"):
--   export AUTOLINK_PLUGIN_INDEX_PATH=/path/to/install/autonomy/share/autolink_plugin_index
--   (or set AUTOLINK_DISTRIBUTION_HOME to install prefix so .../share/autolink_plugin_index is used)
--   export AUTOLINK_PLUGIN_LIB_PATH=/path/to/install/autonomy/lib  -- if plugin .so are not on LD_LIBRARY_PATH

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
    navigators = {"navigate_to_pose", "navigate_through_poses", "navigate_to_docking", "track_to_target", "explore_to_anywhere"},
    navigate_to_pose = {
        plugin = "autonomy_tasks/navigate_to_pose",
        enable_groot_monitoring = false,
        groot_server_port = 1667,
    },
    navigate_through_poses = {
        plugin = "autonomy_tasks/navigate_through_poses",
        enable_groot_monitoring = false,
        groot_server_port = 1669,
    },
    navigate_to_docking = {
        plugin = "autonomy_tasks/navigate_to_docking",
        enable_groot_monitoring = false,
        groot_server_port = 1670,
    },
    track_to_target = {
        plugin = "autonomy_tasks/track_to_target",
        enable_groot_monitoring = false,
        groot_server_port = 1671,
    },
    -- explore_to_anywhere = {
    --     plugin = "autonomy_tasks/explore_to_anywhere",
    --     enable_groot_monitoring = false,
    --     groot_server_port = 1672,
    -- },
    error_code_name_prefixes = {
        "assisted_teleop", "backup", "compute_path", "dock_robot", "drive_on_heading",
        "follow_path", "nav_thru_poses", "nav_to_pose", "route", "spin", "smoother",
        "undock_robot", "wait",
    },
}
