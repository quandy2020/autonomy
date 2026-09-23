# Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

# Sensor driver package. This file builds libautodriver from autodriver/
# and the autodriver process. Chassis sources and vendor libraries are
# added from chassis/. A missing optional SDK does not fail configuration;
# those drivers stay compiled as stubs.

cmake_minimum_required(VERSION 3.20)

project(autodriver VERSION 0.2.0 LANGUAGES CXX)

# Installed autocmake, or the sibling source tree.
find_package(autocmake CONFIG QUIET)
if(NOT autocmake_FOUND)
  set(AUTOCMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autocmake")
  include("${AUTOCMAKE_DIR}/Autocmake.cmake")
endif()

option(AUTODRIVER_BUILD_TEST "Build unit tests" ON)
option(AUTODRIVER_BUILD_EXAMPLES "Build example binaries" OFF)
option(AUTODRIVER_BUILD_DOCS "Build autodriver docs" OFF)
option(AUTODRIVER_WITH_REALSENSE "Build Intel RealSense camera/IMU drivers" OFF)
option(AUTODRIVER_WITH_ORBBEC "Build Orbbec camera/pointcloud drivers" OFF)
option(AUTODRIVER_WITH_RPLIDAR "Build Slamtec RPLidar 2D lidar drivers" OFF)
option(AUTODRIVER_WITH_LIVOX "Build Livox 3D lidar drivers (SDK1+SDK2)" OFF)
option(AUTODRIVER_WITH_GENISOM_L1W
  "Link official GENISOM ZSL-1W HighLevel SDK when found" ON)

# autocmake_test follows BUILD_TESTING. autocmake_build() enables it,
# so turn it off here when unit tests are disabled.
if(DEFINED AUTODRIVER_BUILD_TEST AND NOT AUTODRIVER_BUILD_TEST)
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

autocmake_project()
autocmake_build(QUIT)

# Read by cmake/autodriver_version.cmake.
set(AUTODRIVER_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(AUTODRIVER_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/autodriver")

list(PREPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/module")

include(${AUTODRIVER_ROOT_DIR}/cmake/autodriver_version.cmake)
autodriver_collect_git_version()

set(AUTODRIVER_DEFAULT_CONFIG_DIR "${AUTODRIVER_ROOT_DIR}/config")
file(MAKE_DIRECTORY "${AUTODRIVER_GENERATED_DIR}/conf")
configure_file(
  "${AUTODRIVER_ROOT_DIR}/config/conf.hpp.in"
  "${AUTODRIVER_GENERATED_DIR}/conf/conf.hpp"
  @ONLY)

# Find autolink. 
if(NOT TARGET autolink)
  find_package(autolink REQUIRED)
endif()

# Find automsgs. 
if(NOT TARGET automsgs)
  find_package(automsgs REQUIRED)
endif()

# RealSense imu/camera/pointcloud drivers are added below when the SDK
# is found. device_hub and camera_info stay in this list.
file(GLOB_RECURSE AUTODRIVER_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/autodriver/*.cpp")
list(FILTER AUTODRIVER_SOURCES EXCLUDE REGEX
  "/camera/realsense/(imu_driver|camera_driver|pointcloud_driver)\\.cpp$")

# Build the shared library.
autocmake_library(autodriver SHARED
  SOURCES ${AUTODRIVER_SOURCES}
  DEPENDENCIES 
    Threads::Threads 
    Eigen3::Eigen 
    yaml-cpp 
    autolink 
    automsgs)
set_target_properties(autodriver PROPERTIES SOVERSION "${AUTODRIVER_SOVERSION}")
target_include_directories(autodriver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>)

# Hotplug. Leave AUTODRIVER_HAVE_UDEV undefined when libudev is absent.
find_library(UDEV_LIBRARY udev)
if(UDEV_LIBRARY)
  target_link_libraries(autodriver PRIVATE ${UDEV_LIBRARY})
  target_compile_definitions(autodriver PRIVATE AUTODRIVER_HAVE_UDEV)
endif()

# Orbbec SDK
if(AUTODRIVER_WITH_ORBBEC)
  autocmake_find(OrbbecSDK)
  if(OrbbecSDK_FOUND OR TARGET OrbbecSDK::OrbbecSDK)
    # Some Orbbec packages set the singular OrbbecSDK_LIBRARY only.
    if(NOT OrbbecSDK_LIBRARIES AND OrbbecSDK_LIBRARY)
      set(OrbbecSDK_LIBRARIES "${OrbbecSDK_LIBRARY}")
    endif()
    autocmake_link(autodriver OrbbecSDK)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_ORBBEC)
  endif()
endif()

# RealSense SDK 
if(AUTODRIVER_WITH_REALSENSE)
  autocmake_find(realsense2)
  if(realsense2_FOUND)
    target_sources(autodriver PRIVATE
      autodriver/camera/realsense/imu_driver.cpp
      autodriver/camera/realsense/camera_driver.cpp
      autodriver/camera/realsense/pointcloud_driver.cpp)
    autocmake_link(autodriver realsense2)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_REALSENSE)
  endif()
endif()

# RPLidar SDK
if(AUTODRIVER_WITH_RPLIDAR)
  autocmake_find(RplidarSDK PRIVATE)
  if(RplidarSDK_FOUND)
    autocmake_link(autodriver PRIVATE RplidarSDK)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_RPLIDAR)
  endif()
endif()

# Livox SDK
if(AUTODRIVER_WITH_LIVOX)
  autocmake_find(LivoxSDK2 PRIVATE)
  if(LivoxSDK2_FOUND)
    autocmake_link(autodriver PRIVATE LivoxSDK2)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_LIVOX_SDK2)
  endif()
  autocmake_find(LivoxSDK PRIVATE)
  if(LivoxSDK_FOUND)
    autocmake_link(autodriver PRIVATE LivoxSDK)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_LIVOX_SDK1)
  endif()
endif()

# Chassis drivers
add_subdirectory(chassis)

# Build the autodriver process.
autocmake_binary(autodriver_main
  SOURCES main.cpp options.cpp
  DEPENDENCIES autodriver)
set_target_properties(autodriver_main PROPERTIES OUTPUT_NAME autodriver)
target_include_directories(autodriver_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include)

# Build examples.
if(AUTODRIVER_BUILD_EXAMPLES)
  autocmake_binary(autodriver_demo
    SOURCES examples/demo_main.cpp
    DEPENDENCIES autodriver)
endif()

# Build unit tests.
if(AUTODRIVER_BUILD_TEST)
  autocmake_test(test_time_sync SOURCES test/test_time_sync.cpp DEPENDENCIES autodriver)
  autocmake_test(test_hub SOURCES test/test_hub.cpp DEPENDENCIES autodriver)
  autocmake_test(test_nmea0183 SOURCES test/test_nmea0183.cpp DEPENDENCIES autodriver)
  autocmake_test(test_wit_motion SOURCES test/test_wit_motion.cpp DEPENDENCIES autodriver)
  autocmake_test(test_realsense SOURCES test/test_realsense.cpp DEPENDENCIES autodriver)
  autocmake_test(test_config SOURCES test/test_config.cpp DEPENDENCIES autodriver)
  autocmake_test(test_config_loader SOURCES test/test_config_loader.cpp DEPENDENCIES autodriver)
  autocmake_test(test_manager SOURCES test/test_manager.cpp DEPENDENCIES autodriver)
  autocmake_test(test_stream_lidar_base SOURCES test/test_stream_lidar_base.cpp DEPENDENCIES autodriver)
  autocmake_test(test_pose_feeder SOURCES test/test_pose_feeder.cpp DEPENDENCIES autodriver)
  autocmake_test(test_compensator_calibration SOURCES test/test_compensator_calibration.cpp DEPENDENCIES autodriver)
  autocmake_test(test_lidar_backend_registry SOURCES test/test_lidar_backend_registry.cpp DEPENDENCIES autodriver)
  autocmake_test(test_camera_backend_registry SOURCES test/test_camera_backend_registry.cpp DEPENDENCIES autodriver)
  autocmake_test(test_chassis_backend_registry SOURCES test/test_chassis_backend_registry.cpp DEPENDENCIES autodriver)
  autocmake_test(test_chassis_abstractions SOURCES test/test_chassis_abstractions.cpp DEPENDENCIES autodriver)
  autocmake_test(test_jetauto_kinematics SOURCES test/test_jetauto_kinematics.cpp DEPENDENCIES autodriver autodriver_jetauto)
  autocmake_test(test_l1w_driver SOURCES test/test_l1w_driver.cpp DEPENDENCIES autodriver autodriver_l1w)
  autocmake_test(test_joy_mapper SOURCES test/test_joy_mapper.cpp DEPENDENCIES autodriver)
  autocmake_test(test_canbus_skeleton SOURCES test/test_canbus_skeleton.cpp DEPENDENCIES autodriver)
  autocmake_test(test_skeleton_modules SOURCES test/test_skeleton_modules.cpp DEPENDENCIES autodriver)
endif()

if(AUTODRIVER_BUILD_DOCS)
  add_subdirectory(docs)
endif()

# Each directory installs its own headers or data files.
add_subdirectory(autodriver)
add_subdirectory(config)
add_subdirectory(dag)
add_subdirectory(launch)

autocmake_package()
