# Copyright 2024 The Openbot Authors (duyongquan)
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

# Include autolink_export_plugin function
include("${PROJECT_SOURCE_DIR}/../autolink/cmake/autolink_export_plugin.cmake")

# ============================================================================
# Helper Function: Create and Export Navigator Plugin
# ============================================================================
# Creates a navigator plugin library, links dependencies, exports it, and adds to plugin_libs
# (Same pattern as _create_control_plugin: one source file per plugin, shared description XML)
function(_create_navigator_plugin plugin_name source_file index_name install_subdir)
  set(lib_name "${PROJECT_NAME}_tasks_navigator_${plugin_name}")

  # Convert source file to absolute path if it's relative
  get_filename_component(abs_source_file ${source_file} ABSOLUTE BASE_DIR ${PROJECT_SOURCE_DIR})

  # Skip if source file does not exist (plugin not yet implemented)
  if(NOT EXISTS ${abs_source_file})
    message(STATUS "Navigator plugin skipped (source not found): ${plugin_name} (${abs_source_file})")
    return()
  endif()

  add_library(${lib_name} SHARED ${abs_source_file})

  target_link_libraries(${lib_name} PUBLIC
    ${PROJECT_NAME}
    autolink
  )

  # Get the directory where this file is located
  get_filename_component(PLUGINS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)

  autolink_export_plugin(
    LIBRARY ${lib_name}
    DESCRIPTION_FILE ${PLUGINS_CMAKE_DIR}/navigator_plugins.xml
    INDEX_NAME ${index_name}
    INSTALL_SUBDIR ${install_subdir}
  )

  list(APPEND plugin_libs ${lib_name})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

# ============================================================================
# Navigator Plugins
# ============================================================================

# Navigate To Pose
_create_navigator_plugin(
  "navigate_to_pose"
  "${PROJECT_SOURCE_DIR}/autonomy/tasks/navigator/navigation/navigate_to_pose.cpp"
  "navigate_to_pose"
  "tasks/navigator"
)

# # Navigate Through Poses
# _create_navigator_plugin(
#   "navigate_through_poses"
#   "${PROJECT_SOURCE_DIR}/autonomy/tasks/navigator/navigation/navigate_through_poses.cpp"
#   "navigate_through_poses"
#   "tasks/navigator"
# )

# # Navigate To Docking
# _create_navigator_plugin(
#   "navigate_to_docking"
#   "${PROJECT_SOURCE_DIR}/autonomy/tasks/navigator/docking/dock_navigator.cpp"
#   "navigate_to_docking"
#   "tasks/navigator"
# )

# # Explore To Anywhere
# _create_navigator_plugin(
#   "explore_to_anywhere"
#   "${PROJECT_SOURCE_DIR}/autonomy/tasks/navigator/exploration/explore_to_anywhere.cpp"
#   "explore_to_anywhere"
#   "tasks/navigator"
# )

# # Track To Target
# _create_navigator_plugin(
#   "track_to_target"
#   "${PROJECT_SOURCE_DIR}/autonomy/tasks/navigator/track/track_to_target.cpp"
#   "track_to_target"
#   "tasks/navigator"
# )
