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
  config_loader.cpp
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
  # all sensor modules compiled directly into the shared library
  modules.cpp
  hardware/serial_imu_driver.cpp
  hardware/can_imu_driver.cpp
  hardware/serial_gps_driver.cpp
  hardware/can_gps_driver.cpp
  bridge/publisher.cpp
)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(autodriver SHARED ${AUTODRIVER_SOURCES})

if(NOT TARGET autolink OR NOT TARGET automsgs)
  message(FATAL_ERROR
    "autodriver requires autolink and automsgs targets to already be defined. "
    "When building standalone, ensure autolink and automsgs are installed and "
    "findable. When building inside the autonomy super-project, use "
    "-DBUILD_AUTODRIVER=ON and make sure autodriver is added after automsgs.")
endif()

target_include_directories(autodriver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<BUILD_INTERFACE:${AUTODRIVER_GENERATED_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(autodriver PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(autodriver PUBLIC Threads::Threads autolink automsgs
  yaml-cpp)
target_compile_definitions(autodriver PUBLIC
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

