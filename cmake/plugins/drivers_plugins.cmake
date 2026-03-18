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

# Include autolink_export_plugin function
include("${PROJECT_SOURCE_DIR}/autolink/cmake/autolink_export_plugin.cmake")

# ============================================================================
# Helper Function: Create and Export Driver Plugin
# ============================================================================
# Creates a driver plugin library, links dependencies, exports it, and adds to plugin_libs
function(_create_driver_plugin plugin_name source_file index_name install_subdir)
  set(lib_name "${PROJECT_NAME}_driver_${plugin_name}")
  
  add_library(${lib_name} SHARED ${source_file})
  
  target_link_libraries(${lib_name} PUBLIC
    ${PROJECT_NAME}
    autolink
  )
  
  # Get the directory where this file is located
  get_filename_component(PLUGINS_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
  
  autolink_export_plugin(
    LIBRARY ${lib_name}
    DESCRIPTION_FILE ${PLUGINS_CMAKE_DIR}/drivers_plugins.xml
    INDEX_NAME ${index_name}
    INSTALL_SUBDIR ${install_subdir}
  )
  
  list(APPEND plugin_libs ${lib_name})
  set(plugin_libs ${plugin_libs} PARENT_SCOPE)
endfunction()

# ============================================================================
# Camera Driver Plugins
# ============================================================================

# RealSense D435i Driver
_create_driver_plugin(
  "camera_realsense_d435i"
  "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/camera/realsense_d435i.cpp"
  "camera_realsense_d435i"
  "driver/sensor/camera"
)

# ============================================================================
# IMU Driver Plugins
# ============================================================================

# MPU6050 Driver
_create_driver_plugin(
  "imu_mpu6050"
  "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/imu/mpu_6050.cpp"
  "imu_mpu6050"
  "driver/sensor/imu"
)

# ============================================================================
# Range Driver Plugins
# ============================================================================

# TODO: 添加测距传感器驱动插件
# 当有具体的测距传感器驱动实现时，可以在这里添加，例如：
# _create_driver_plugin(
#   "range_hcsr04"
#   "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/range/hcsr04.cpp"
#   "range_hcsr04"
#   "driver/sensor/range"
# )

# ============================================================================
# Lidar Driver Plugins
# ============================================================================

# TODO: 添加激光雷达驱动插件
# 当有具体的激光雷达驱动实现时，可以在这里添加，例如：
# _create_driver_plugin(
#   "lidar_rplidar_a1"
#   "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/lidar/rplidar_a1.cpp"
#   "lidar_rplidar_a1"
#   "driver/sensor/lidar"
# )

# ============================================================================
# GPS Driver Plugins
# ============================================================================

# TODO: 添加GPS驱动插件
# 当有具体的GPS驱动实现时，可以在这里添加，例如：
# _create_driver_plugin(
#   "gps_ublox_m8n"
#   "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/gps/ublox_m8n.cpp"
#   "gps_ublox_m8n"
#   "driver/sensor/gps"
# )
