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
# Helper Function: Create and Export Plugin
# ============================================================================
# Creates a plugin library, links dependencies, exports it, and adds to plugin_libs
function(_create_control_plugin plugin_name source_file index_name install_subdir)
  set(lib_name "${PROJECT_NAME}_control_${plugin_name}")
  
  # Convert source file to absolute path if it's relative
  get_filename_component(abs_source_file ${source_file} ABSOLUTE BASE_DIR ${PROJECT_SOURCE_DIR})
  
  # Check if source file exists
  if(NOT EXISTS ${abs_source_file})
    message(FATAL_ERROR "Plugin source file not found: ${abs_source_file} (resolved from ${source_file})")
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
    DESCRIPTION_FILE ${PLUGINS_CMAKE_DIR}/controller_plugins.xml
    INDEX_NAME ${index_name}
    INSTALL_SUBDIR ${install_subdir}
  )
  
  list(APPEND plugin_libs ${lib_name})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

# ============================================================================
# Controller Plugins
# ============================================================================

# Graceful Controller
_create_control_plugin(
  "graceful_controller"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/graceful_controller/graceful_controller.cpp"
  "graceful_controller"
  "control/controller"
)

# Regulated Pure Pursuit Controller
_create_control_plugin(
  "regulated_pure_pursuit_controller"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/pure_pursuit_controller/regulated_pure_pursuit_controller.cpp"
  "regulated_pure_pursuit_controller"
  "control/controller"
)

# MPPI Controller
_create_control_plugin(
  "mppi_controller"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/controller.cpp"
  "mppi_controller"
  "control/controller"
)

# ============================================================================
# MPPI Controller Critics
# ============================================================================

set(MPPI_CRITICS
  constraint_critic
  cost_critic
  goal_angle_critic
  goal_critic
  obstacles_critic
  path_align_critic
  path_angle_critic
  path_follow_critic
  prefer_forward_critic
  twirling_critic
  velocity_deadband_critic
)

foreach(critic ${MPPI_CRITICS})
  _create_control_plugin(
    "mppi_controller_${critic}"
    "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/${critic}.cpp"
    "mppi_${critic}"
    "control/controller/mppi_controller/critics"
  )
endforeach()

# ============================================================================
# Goal Checkers
# ============================================================================

set(GOAL_CHECKERS
  simple_goal_checker
  position_goal_checker
  stopped_goal_checker
)

foreach(checker ${GOAL_CHECKERS})
  _create_control_plugin(
    "${checker}"
    "${PROJECT_SOURCE_DIR}/autonomy/control/checker/${checker}.cpp"
    "${checker}"
    "control/checker"
  )
endforeach()

# ============================================================================
# Progress Checkers
# ============================================================================

set(PROGRESS_CHECKERS
  simple_progress_checker
  pose_progress_checker
)

foreach(checker ${PROGRESS_CHECKERS})
  _create_control_plugin(
    "${checker}"
    "${PROJECT_SOURCE_DIR}/autonomy/control/checker/${checker}.cpp"
    "${checker}"
    "control/checker"
  )
endforeach()
