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
include("${PROJECT_SOURCE_DIR}/autolink/cmake/autolink_export_plugin.cmake")

# ============================================================================
# Helper Function: Create and Export Planner Plugin
# ============================================================================
# Creates a planner plugin library, links dependencies, exports it, and adds to plugin_libs
# Args:
#   plugin_name: Name of the plugin (e.g., "navfn_planner")
#   index_name: Index name for the plugin (e.g., "navfn_planner")
#   install_subdir: Installation subdirectory (e.g., "planning/planner")
#   source_files: List of source files (variable arguments)
function(_create_planner_plugin plugin_name index_name install_subdir)
  set(lib_name "${PROJECT_NAME}_planning_${plugin_name}")
  
  # Collect all source files from remaining arguments
  set(source_files ${ARGN})
  
  add_library(${lib_name} SHARED ${source_files})
  
  target_link_libraries(${lib_name} PUBLIC
    ${PROJECT_NAME}
    autolink
  )
  
  # Get the directory where this file is located
  get_filename_component(PLUGINS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
  
  autolink_export_plugin(
    LIBRARY ${lib_name}
    DESCRIPTION_FILE ${PLUGINS_CMAKE_DIR}/planner_plugins.xml
    INDEX_NAME ${index_name}
    INSTALL_SUBDIR ${install_subdir}
  )
  
  list(APPEND plugin_libs ${lib_name})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

# ============================================================================
# Planner Plugins
# ============================================================================

# Navfn Planner
_create_planner_plugin(
  "navfn_planner"
  "navfn_planner"
  "planning/planner"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/navfn/navfn.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/navfn/navfn_planner.cpp"
)

# Theta Star Planner
_create_planner_plugin(
  "theta_star_planner"
  "theta_star_planner"
  "planning/planner"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/theta_star/theta_star.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/theta_star/theta_star_planner.cpp"
)
