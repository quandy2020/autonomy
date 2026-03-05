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
# Helper Function: Create and Export Map Plugin
# ============================================================================
# Creates a map plugin library, links dependencies, exports it, and adds to plugin_libs
function(_create_map_plugin plugin_name source_file index_name install_subdir)
  set(lib_name "${PROJECT_NAME}_map_${plugin_name}")
  
  add_library(${lib_name} SHARED ${source_file})
  
  target_link_libraries(${lib_name} PUBLIC
    ${PROJECT_NAME}
    autolink
  )
  
  # Get the directory where this file is located
  get_filename_component(PLUGINS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
  
  autolink_export_plugin(
    LIBRARY ${lib_name}
    DESCRIPTION_FILE ${PLUGINS_CMAKE_DIR}/map_plugins.xml
    INDEX_NAME ${index_name}
    INSTALL_SUBDIR ${install_subdir}
  )
  
  list(APPEND plugin_libs ${lib_name})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

# ============================================================================
# Costmap Layer Plugins
# ============================================================================

set(COSTMAP_LAYERS
  layers_voxel_layer
  layers_static_layer
  layers_obstacle_layer
  layers_inflation_layer
  layers_denoise_layer
  layers_range_sensor_layer
)

foreach(layer ${COSTMAP_LAYERS})
  # Extract the layer name (e.g., "voxel_layer" from "layers_voxel_layer")
  string(REPLACE "layers_" "" layer_name ${layer})
  
  _create_map_plugin(
    "${layer}"
    "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/${layer_name}.cpp"
    "${layer_name}"
    "map/costmap_2d/layers"
  )
endforeach()

# ============================================================================
# Costmap Filter Plugins
# ============================================================================

set(COSTMAP_FILTERS
  filters_binary_filter
  filters_speed_filter
  filters_keepout_filter
)

foreach(filter ${COSTMAP_FILTERS})
  # Extract the filter name (e.g., "binary_filter" from "filters_binary_filter")
  string(REPLACE "filters_" "" filter_name ${filter})
  
  _create_map_plugin(
    "${filter}"
    "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/filters/${filter_name}.cpp"
    "${filter_name}"
    "map/costmap_2d/filters"
  )
endforeach()
