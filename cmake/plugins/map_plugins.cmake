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


################################ layers plugins #################################
# voxel_layer
add_library(${PROJECT_NAME}_map_layers_voxel_layer SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/voxel_layer.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_layers_voxel_layer)

# static_layer
add_library(${PROJECT_NAME}_map_layers_static_layer SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/static_layer.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_layers_static_layer)

# obstacle_layer
add_library(${PROJECT_NAME}_map_layers_obstacle_layer SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/obstacle_layer.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_layers_obstacle_layer)

# inflation_layer
add_library(${PROJECT_NAME}_map_layers_inflation_layer SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/inflation_layer.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_layers_inflation_layer)

# denoise_layer
add_library(${PROJECT_NAME}_map_layers_denoise_layer SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/denoise_layer.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_layers_denoise_layer)

# range_sensor_layer
add_library(${PROJECT_NAME}_map_layers_range_sensor_layer SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/layers/range_sensor_layer.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_layers_range_sensor_layer)


################################ filters plugins #################################
# binary_filter
add_library(${PROJECT_NAME}_map_filters_binary_filter SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/filters/binary_filter.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_filters_binary_filter)

# speed_filter
add_library(${PROJECT_NAME}_map_filters_speed_filter SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/filters/speed_filter.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_filters_speed_filter)

# keepout_filter
add_library(${PROJECT_NAME}_map_filters_keepout_filter SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/map/costmap_2d/filters/keepout_filter.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_map_filters_keepout_filter)