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

################################ Camera Driver Plugins #################################

# RealSense D435i Driver Plugin
add_library(${PROJECT_NAME}_driver_camera_realsense_d435i SHARED
    "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/camera/realsense_d435i.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_driver_camera_realsense_d435i)

################################ IMU Driver Plugins ####################################

# MPU6050 Driver Plugin
add_library(${PROJECT_NAME}_driver_imu_mpu6050 SHARED
    "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/imu/mpu_6050.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_driver_imu_mpu6050)

################################ Range Driver Plugins ##################################

# TODO: 添加测距传感器驱动插件
# 当有具体的测距传感器驱动实现时，可以在这里添加，例如：
# add_library(${PROJECT_NAME}_range_hcsr04_plugin SHARED
#     "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/range/hcsr04.cpp"
# )
# target_include_directories(${PROJECT_NAME}_range_hcsr04_plugin
#     PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(${PROJECT_NAME}_range_hcsr04_plugin ${PROJECT_NAME})
# list(APPEND plugin_libs ${PROJECT_NAME}_range_hcsr04_plugin)

################################ Lidar Driver Plugins ##################################

# TODO: 添加激光雷达驱动插件
# 当有具体的激光雷达驱动实现时，可以在这里添加，例如：
# add_library(${PROJECT_NAME}_lidar_rplidar_a1_plugin SHARED
#     "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/lidar/rplidar_a1.cpp"
# )
# target_include_directories(${PROJECT_NAME}_lidar_rplidar_a1_plugin
#     PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(${PROJECT_NAME}_lidar_rplidar_a1_plugin ${PROJECT_NAME})
# list(APPEND plugin_libs ${PROJECT_NAME}_lidar_rplidar_a1_plugin)

################################ GPS Driver Plugins ###################################

# TODO: 添加GPS驱动插件
# 当有具体的GPS驱动实现时，可以在这里添加，例如：
# add_library(${PROJECT_NAME}_gps_ublox_m8n_plugin SHARED
#     "${PROJECT_SOURCE_DIR}/autonomy/driver/sensor/gps/ublox_m8n.cpp"
# )
# target_include_directories(${PROJECT_NAME}_gps_ublox_m8n_plugin
#     PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
# )
# target_link_libraries(${PROJECT_NAME}_gps_ublox_m8n_plugin ${PROJECT_NAME})
# list(APPEND plugin_libs ${PROJECT_NAME}_gps_ublox_m8n_plugin)

#########################################################################################
