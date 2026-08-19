# Copyright 2026 Autodriver contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(AUTODRIVER_SOURCES
  config.cpp
  sensor_manager.cpp
  driver_params.cpp
  hardware/serial_port.cpp
  hardware/can_socket.cpp
  hardware/realsense_device_hub.cpp
  hardware/nmea_0183.cpp
  hardware/wit_motion_parser.cpp
  time_synchronization.cpp
  sample_buffer.cpp
  sensor_hub.cpp
)

find_package(Threads REQUIRED)

add_library(autodriver ${AUTODRIVER_SOURCES})

if(NOT TARGET autolink OR NOT TARGET automsgs)
  message(FATAL_ERROR
    "autodriver requires autolink and automsgs; configure the autonomy root "
    "with -DBUILD_AUTODRIVER=ON")
endif()

target_include_directories(autodriver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(autodriver PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(autodriver PUBLIC Threads::Threads autolink automsgs)
target_compile_definitions(autodriver PUBLIC
  AUTODRIVER_PLUGIN_DIR="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  AUTODRIVER_SHARED_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

if(UNIX AND NOT APPLE)
  find_library(UDEV_LIBRARY udev)
  if(UDEV_LIBRARY)
    target_link_libraries(autodriver PRIVATE ${UDEV_LIBRARY})
    target_compile_definitions(autodriver PRIVATE AUTODRIVER_HAVE_UDEV)
  endif()
endif()

if(AUTODRIVER_WITH_REALSENSE)
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    message(STATUS "autodriver: librealsense2 ${realsense2_VERSION} enabled")
    target_sources(autodriver PRIVATE
      hardware/realsense_imu_driver.cpp
      hardware/realsense_camera_driver.cpp)
    target_link_libraries(autodriver PUBLIC realsense2::realsense2)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_REALSENSE)
  else()
    message(STATUS "autodriver: librealsense2 not found; RealSense backends disabled")
  endif()
endif()

function(autodriver_add_plugin name)
  add_library(${name} SHARED ${ARGN})
  target_link_libraries(${name} PRIVATE autodriver)
  target_include_directories(${name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  set_target_properties(${name} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endfunction()

autodriver_add_plugin(autodriver_lidar plugins/lidar/lidar_module.cpp)
autodriver_add_plugin(autodriver_range plugins/range/range_module.cpp)
autodriver_add_plugin(autodriver_gps
  plugins/gps/gps_module.cpp
  hardware/serial_gps_driver.cpp
  hardware/can_gps_driver.cpp)
autodriver_add_plugin(autodriver_imu
  plugins/imu/imu_module.cpp
  hardware/serial_imu_driver.cpp
  hardware/can_imu_driver.cpp)
autodriver_add_plugin(autodriver_camera plugins/camera/camera_module.cpp)
