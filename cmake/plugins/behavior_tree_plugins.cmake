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

# ============================================================================
# Behavior Tree Plugins Configuration
# ============================================================================
# Note: Behavior tree plugins use BT_REGISTER_NODES (from BehaviorTree.CPP)
#       instead of CLASS_LOADER_REGISTER_CLASS, so they don't need:
#       - XML description files (like controller_plugins.xml, map_plugins.xml, etc.)
#       - autolink_export_plugin() function
#       Plugins are registered directly in the source code using BT_REGISTER_NODES macro.

# ============================================================================
# Helper Function: Create Behavior Tree Plugin
# ============================================================================
# Creates a behavior tree plugin library, links dependencies, and adds to plugin_libs.
# BT_PLUGIN_EXPORT is required so BT_REGISTER_NODES exports BT_RegisterNodesFromPlugin (see bt_factory.h).
function(_create_bt_plugin plugin_category plugin_name source_file)
  set(lib_name "${PROJECT_NAME}_behavior_tree_${plugin_category}_${plugin_name}")
  
  add_library(${lib_name} SHARED ${source_file})
  
  target_compile_definitions(${lib_name} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${lib_name} PUBLIC
    ${PROJECT_NAME}
    autolink
  )
  
  list(APPEND plugin_libs ${lib_name})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

# ============================================================================
# Behavior Tree Action Plugins
# ============================================================================

# Selector nodes
set(BT_SELECTOR_NODES
  controller_selector_node
  planner_selector_node
  goal_checker_selector_node
  progress_checker_selector_node
  smoother_selector_node
)

foreach(node ${BT_SELECTOR_NODES})
  _create_bt_plugin(
    "action"
    "${node}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${node}.cpp"
  )
endforeach()

# Navigation actions
set(BT_NAVIGATION_ACTIONS
  navigate_to_pose_action
  navigate_through_poses_action
  follow_path_action
  compute_path_to_pose_action
  compute_path_through_poses_action
  compute_route_action
  compute_and_track_route_action
  compute_and_track_route_cancel_node
)

foreach(action ${BT_NAVIGATION_ACTIONS})
  _create_bt_plugin(
    "action"
    "${action}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${action}.cpp"
  )
endforeach()

# Path manipulation actions
set(BT_PATH_ACTIONS
  smooth_path_action
  truncate_path_action
  truncate_path_local_action
  concatenate_paths_action
)

foreach(action ${BT_PATH_ACTIONS})
  _create_bt_plugin(
    "action"
    "${action}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${action}.cpp"
  )
endforeach()

# Basic behavior actions
set(BT_BASIC_ACTIONS
  back_up_action
  back_up_cancel_node
  spin_action
  spin_cancel_node
  wait_action
  wait_cancel_node
  drive_on_heading_action
  drive_on_heading_cancel_node
  assisted_teleop_action
  assisted_teleop_cancel_node
  controller_cancel_node
)

foreach(action ${BT_BASIC_ACTIONS})
  _create_bt_plugin(
    "action"
    "${action}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${action}.cpp"
  )
endforeach()

# Goal manipulation actions
set(BT_GOAL_ACTIONS
  append_goal_pose_to_goals_action
  extract_route_nodes_as_goals_action
  remove_in_collision_goals_action
  remove_passed_goals_action
  get_next_few_goals_action
)

foreach(action ${BT_GOAL_ACTIONS})
  _create_bt_plugin(
    "action"
    "${action}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${action}.cpp"
  )
endforeach()

# Utility actions
set(BT_UTILITY_ACTIONS
  get_current_pose_action
  get_pose_from_path_action
)

foreach(action ${BT_UTILITY_ACTIONS})
  _create_bt_plugin(
    "action"
    "${action}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${action}.cpp"
  )
endforeach()

# Service actions
set(BT_SERVICE_ACTIONS
  clear_costmap_service
  reinitialize_global_localization_service
)

foreach(action ${BT_SERVICE_ACTIONS})
  _create_bt_plugin(
    "action"
    "${action}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/action/${action}.cpp"
  )
endforeach()

# ============================================================================
# Behavior Tree Condition Plugins
# ============================================================================

set(BT_CONDITIONS
  are_error_codes_present_condition
  are_poses_near_condition
  distance_traveled_condition
  globally_updated_goal_condition
  goal_reached_condition
  goal_updated_condition
  initial_pose_received_condition
  is_battery_charging_condition
  is_battery_low_condition
  is_path_valid_condition
  is_stopped_condition
  is_stuck_condition
  path_expiring_timer_condition
  time_expired_condition
  transform_available_condition
  would_a_controller_recovery_help_condition
  would_a_planner_recovery_help_condition
  would_a_route_recovery_help_condition
  would_a_smoother_recovery_help_condition
)

foreach(condition ${BT_CONDITIONS})
  _create_bt_plugin(
    "condition"
    "${condition}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/condition/${condition}.cpp"
  )
endforeach()

# ============================================================================
# Behavior Tree Control Plugins
# ============================================================================

set(BT_CONTROL_NODES
  pipeline_sequence
  recovery_node
  round_robin_node
  persistent_sequence
  nonblocking_sequence
)

foreach(node ${BT_CONTROL_NODES})
  _create_bt_plugin(
    "control"
    "${node}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/control/${node}.cpp"
  )
endforeach()

# ============================================================================
# Behavior Tree Decorator Plugins
# ============================================================================

set(BT_DECORATORS
  distance_controller
  goal_updated_controller
  goal_updater_node
  path_longer_on_approach
  rate_controller
  single_trigger_node
  speed_controller
)

foreach(decorator ${BT_DECORATORS})
  _create_bt_plugin(
    "decorator"
    "${decorator}"
    "${PROJECT_SOURCE_DIR}/autonomy/tasks/behavior_tree/plugins/decorator/${decorator}.cpp"
  )
endforeach()
