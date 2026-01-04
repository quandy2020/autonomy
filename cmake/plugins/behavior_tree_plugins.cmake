# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

################################ behavior tree plugins(action) #################################

# Selector nodes
# controller_selector_node
add_library(${PROJECT_NAME}_behavior_tree_action_controller_selector_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/controller_selector_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_controller_selector_node)

# planner_selector_node
add_library(${PROJECT_NAME}_behavior_tree_action_planner_selector_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/planner_selector_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_planner_selector_node)

# goal_checker_selector_node
add_library(${PROJECT_NAME}_behavior_tree_action_goal_checker_selector_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/goal_checker_selector_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_goal_checker_selector_node)

# progress_checker_selector_node
add_library(${PROJECT_NAME}_behavior_tree_action_progress_checker_selector_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/progress_checker_selector_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_progress_checker_selector_node)

# smoother_selector_node
add_library(${PROJECT_NAME}_behavior_tree_action_smoother_selector_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/smoother_selector_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_smoother_selector_node)

# Navigation actions
# navigate_to_pose_action
add_library(${PROJECT_NAME}_behavior_tree_action_navigate_to_pose_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/navigate_to_pose_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_navigate_to_pose_action)

# navigate_through_poses_action
add_library(${PROJECT_NAME}_behavior_tree_action_navigate_through_poses_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/navigate_through_poses_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_navigate_through_poses_action)

# follow_path_action
add_library(${PROJECT_NAME}_behavior_tree_action_follow_path_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/follow_path_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_follow_path_action)

# compute_path_to_pose_action
add_library(${PROJECT_NAME}_behavior_tree_action_compute_path_to_pose_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/compute_path_to_pose_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_compute_path_to_pose_action)

# compute_path_through_poses_action
add_library(${PROJECT_NAME}_behavior_tree_action_compute_path_through_poses_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/compute_path_through_poses_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_compute_path_through_poses_action)

# compute_route_action
add_library(${PROJECT_NAME}_behavior_tree_action_compute_route_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/compute_route_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_compute_route_action)

# compute_and_track_route_action
add_library(${PROJECT_NAME}_behavior_tree_action_compute_and_track_route_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/compute_and_track_route_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_compute_and_track_route_action)

# compute_and_track_route_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_compute_and_track_route_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/compute_and_track_route_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_compute_and_track_route_cancel_node)

# Path manipulation actions
# smooth_path_action
add_library(${PROJECT_NAME}_behavior_tree_action_smooth_path_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/smooth_path_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_smooth_path_action)

# truncate_path_action
add_library(${PROJECT_NAME}_behavior_tree_action_truncate_path_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/truncate_path_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_truncate_path_action)

# truncate_path_local_action
add_library(${PROJECT_NAME}_behavior_tree_action_truncate_path_local_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/truncate_path_local_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_truncate_path_local_action)

# concatenate_paths_action
add_library(${PROJECT_NAME}_behavior_tree_action_concatenate_paths_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/concatenate_paths_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_concatenate_paths_action)

# Basic behavior actions
# back_up_action
add_library(${PROJECT_NAME}_behavior_tree_action_back_up_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/back_up_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_back_up_action)

# back_up_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_back_up_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/back_up_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_back_up_cancel_node)

# spin_action
add_library(${PROJECT_NAME}_behavior_tree_action_spin_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/spin_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_spin_action)

# spin_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_spin_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/spin_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_spin_cancel_node)

# wait_action
add_library(${PROJECT_NAME}_behavior_tree_action_wait_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/wait_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_wait_action)

# wait_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_wait_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/wait_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_wait_cancel_node)

# drive_on_heading_action
add_library(${PROJECT_NAME}_behavior_tree_action_drive_on_heading_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/drive_on_heading_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_drive_on_heading_action)

# drive_on_heading_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_drive_on_heading_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/drive_on_heading_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_drive_on_heading_cancel_node)

# assisted_teleop_action
add_library(${PROJECT_NAME}_behavior_tree_action_assisted_teleop_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/assisted_teleop_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_assisted_teleop_action)

# assisted_teleop_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_assisted_teleop_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/assisted_teleop_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_assisted_teleop_cancel_node)

# controller_cancel_node
add_library(${PROJECT_NAME}_behavior_tree_action_controller_cancel_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/controller_cancel_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_controller_cancel_node)

# Goal manipulation actions
# append_goal_pose_to_goals_action
add_library(${PROJECT_NAME}_behavior_tree_action_append_goal_pose_to_goals_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/append_goal_pose_to_goals_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_append_goal_pose_to_goals_action)

# extract_route_nodes_as_goals_action
add_library(${PROJECT_NAME}_behavior_tree_action_extract_route_nodes_as_goals_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/extract_route_nodes_as_goals_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_extract_route_nodes_as_goals_action)

# remove_in_collision_goals_action
add_library(${PROJECT_NAME}_behavior_tree_action_remove_in_collision_goals_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/remove_in_collision_goals_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_remove_in_collision_goals_action)

# remove_passed_goals_action
add_library(${PROJECT_NAME}_behavior_tree_action_remove_passed_goals_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/remove_passed_goals_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_remove_passed_goals_action)

# get_next_few_goals_action
add_library(${PROJECT_NAME}_behavior_tree_action_get_next_few_goals_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/get_next_few_goals_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_get_next_few_goals_action)

# Utility actions
# get_current_pose_action
add_library(${PROJECT_NAME}_behavior_tree_action_get_current_pose_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/get_current_pose_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_get_current_pose_action)

# get_pose_from_path_action
add_library(${PROJECT_NAME}_behavior_tree_action_get_pose_from_path_action SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/get_pose_from_path_action.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_get_pose_from_path_action)

# Service actions
# clear_costmap_service
add_library(${PROJECT_NAME}_behavior_tree_action_clear_costmap_service SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/clear_costmap_service.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_clear_costmap_service)

# reinitialize_global_localization_service
add_library(${PROJECT_NAME}_behavior_tree_action_reinitialize_global_localization_service SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/reinitialize_global_localization_service.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_action_reinitialize_global_localization_service)

################################ behavior tree plugins(condition) #################################
# are_error_codes_present_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_are_error_codes_present_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/are_error_codes_present_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_are_error_codes_present_condition)

# are_poses_near_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_are_poses_near_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/are_poses_near_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_are_poses_near_condition)

# distance_traveled_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_distance_traveled_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/distance_traveled_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_distance_traveled_condition)

# globally_updated_goal_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_globally_updated_goal_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/globally_updated_goal_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_globally_updated_goal_condition)

# goal_reached_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_goal_reached_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/goal_reached_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_goal_reached_condition)

# goal_updated_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_goal_updated_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/goal_updated_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_goal_updated_condition)

# initial_pose_received_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_initial_pose_received_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/initial_pose_received_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_initial_pose_received_condition)

# is_battery_charging_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_is_battery_charging_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/is_battery_charging_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_is_battery_charging_condition)

# is_battery_low_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_is_battery_low_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/is_battery_low_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_is_battery_low_condition)

# is_path_valid_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_is_path_valid_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/is_path_valid_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_is_path_valid_condition)

# is_stopped_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_is_stopped_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/is_stopped_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_is_stopped_condition)

# is_stuck_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_is_stuck_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/is_stuck_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_is_stuck_condition)

# path_expiring_timer_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_path_expiring_timer_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/path_expiring_timer_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_path_expiring_timer_condition)

# time_expired_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_time_expired_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/time_expired_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_time_expired_condition)

# transform_available_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_transform_available_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/transform_available_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_transform_available_condition)

# would_a_controller_recovery_help_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_would_a_controller_recovery_help_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/would_a_controller_recovery_help_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_would_a_controller_recovery_help_condition)

# would_a_planner_recovery_help_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_would_a_planner_recovery_help_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/would_a_planner_recovery_help_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_would_a_planner_recovery_help_condition)

# would_a_route_recovery_help_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_would_a_route_recovery_help_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/would_a_route_recovery_help_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_would_a_route_recovery_help_condition)

# would_a_smoother_recovery_help_condition
add_library(${PROJECT_NAME}_behavior_tree_condition_would_a_smoother_recovery_help_condition SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/would_a_smoother_recovery_help_condition.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_condition_would_a_smoother_recovery_help_condition)

################################ behavior tree plugins(control) #################################
# pipeline_sequence
add_library(${PROJECT_NAME}_behavior_tree_control_pipeline_sequence SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/pipeline_sequence.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_control_pipeline_sequence)

# recovery_node
add_library(${PROJECT_NAME}_behavior_tree_control_recovery_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/recovery_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_control_recovery_node)

# round_robin_node
add_library(${PROJECT_NAME}_behavior_tree_control_round_robin_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/round_robin_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_control_round_robin_node)

# persistent_sequence
add_library(${PROJECT_NAME}_behavior_tree_control_persistent_sequence SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/persistent_sequence.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_control_persistent_sequence)

# # pause_resume_controller
# add_library(${PROJECT_NAME}_behavior_tree_control_pause_resume_controller SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/pause_resume_controller.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_control_pause_resume_controller)

# nonblocking_sequence
add_library(${PROJECT_NAME}_behavior_tree_control_nonblocking_sequence SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/nonblocking_sequence.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_control_nonblocking_sequence)

################################ behavior tree plugins(decorator) #################################
# distance_controller
add_library(${PROJECT_NAME}_behavior_tree_decorator_distance_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/distance_controller.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_distance_controller)

# goal_updated_controller
add_library(${PROJECT_NAME}_behavior_tree_decorator_goal_updated_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/goal_updated_controller.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_goal_updated_controller)

# goal_updater_node
add_library(${PROJECT_NAME}_behavior_tree_decorator_goal_updater_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/goal_updater_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_goal_updater_node)

# path_longer_on_approach
add_library(${PROJECT_NAME}_behavior_tree_decorator_path_longer_on_approach SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/path_longer_on_approach.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_path_longer_on_approach)

# rate_controller
add_library(${PROJECT_NAME}_behavior_tree_decorator_rate_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/rate_controller.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_rate_controller)

# single_trigger_node
add_library(${PROJECT_NAME}_behavior_tree_decorator_single_trigger_node SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/single_trigger_node.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_single_trigger_node)

# speed_controller
add_library(${PROJECT_NAME}_behavior_tree_decorator_speed_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/speed_controller.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_behavior_tree_decorator_speed_controller)
