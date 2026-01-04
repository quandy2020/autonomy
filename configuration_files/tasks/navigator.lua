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

AUTONOMY_TASK_NAVIGATOR = {
    global_frame = "map",
    robot_base_frame = "base_link",
    odom_topic = "odom",
    bt_loop_duration = 10,
    filter_duration = 0.3,
    default_server_timeout = 20,
    wait_for_service_timeout = 1000,
    service_introspection_mode = "disabled",
    -- navigator types
    --  navigate_to_pose
    --  navigate_through_poses
    --  navigate_to_docking
    --  track_to_target
    --  explore_to_anywhere
    navigators = {"navigate_to_pose", "navigate_through_poses", "navigate_to_docking", "track_to_target", "explore_to_anywhere"},
    navigate_to_pose = {
        plugin = "nav2_bt_navigator::NavigateToPoseNavigator",
        enable_groot_monitoring = false,
        groot_server_port = 1667,
    },
    navigate_through_poses = {
        plugin = "nav2_bt_navigator::NavigateThroughPosesNavigator",
        enable_groot_monitoring = false,
        groot_server_port = 1669,
    },
    navigate_to_docking = {

    },
    track_to_target = {

    },
    explore_to_anywhere = {

    },
    -- 'default_nav_through_poses_bt_xml' and 'default_nav_to_pose_bt_xml' are use defaults:
    -- nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml
    -- nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml
    -- They can be set here or via a RewrittenYaml remap from a parent launch file to Nav2.

    -- plugin_lib_names is used to add custom BT plugins to the executor (vector of strings).
    -- Built-in plugins are added automatically
    -- plugin_lib_names = {},

    error_code_name_prefixes = {
        "assisted_teleop",
        "backup",
        "compute_path",
        "dock_robot",
        "drive_on_heading",
        "follow_path",
        "nav_thru_poses",
        "nav_to_pose",
        "route",
        "spin",
        "smoother",
        "undock_robot",
        "wait",
    },
}
