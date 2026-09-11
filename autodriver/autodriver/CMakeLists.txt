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

set(AUTODRIVER_SOURCES
  config.cpp
  config_loader.cpp
  sensor_manager.cpp
  driver_params.cpp
  common/serial_port.cpp
  common/stream.cpp
  common/can_socket.cpp
  common/calibration.cpp
  camera/orbbec/camera_info.cpp
  camera/orbbec/device_hub.cpp
  camera/orbbec/camera_driver.cpp
  camera/orbbec/pointcloud_driver.cpp
  # Pure helpers + hub stubs (no librealsense required at link time).
  camera/realsense/device_hub.cpp
  camera/realsense/camera_info.cpp
  smartereye/camera_driver.cpp
  radar/conti/driver.cpp
  microphone/respeaker_driver.cpp
  gps/nmea_0183.cpp
  gps/parser/parser.cpp
  imu/wit_motion_parser.cpp
  time_synchronization.cpp
  sample_buffer.cpp
  sensor_hub.cpp
  # all sensor modules compiled directly into the shared library
  modules.cpp
  imu/serial_imu_driver.cpp
  imu/can_imu_driver.cpp
  gps/serial_gps_driver.cpp
  gps/can_gps_driver.cpp
  lidar/motion_compensator.cpp
  lidar/pose_buffer.cpp
  lidar/beam_calibration_yaml.cpp
  lidar/velodyne/calibration.cpp
  lidar/velodyne/convert.cpp
  lidar/velodyne/udp_driver.cpp
  lidar/hesai/convert.cpp
  lidar/hesai/calibration.cpp
  lidar/hesai/udp_driver.cpp
  lidar/livox/convert.cpp
  lidar/livox/sdk1_driver.cpp
  lidar/livox/sdk2_driver.cpp
  lidar/livox/driver.cpp
  lidar/rplidar/convert.cpp
  lidar/rplidar/serial_driver.cpp
  lidar/vendor_stubs.cpp
  bridge/publisher.cpp
  bridge/pose_feeder.cpp
)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(autodriver SHARED ${AUTODRIVER_SOURCES})
set_target_properties(autodriver PROPERTIES
  VERSION "${AUTODRIVER_VERSION}"
  SOVERSION "${AUTODRIVER_SOVERSION}")

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
  yaml-cpp Eigen3::Eigen)
target_compile_definitions(autodriver PUBLIC
  AUTODRIVER_SHARED_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

if(UNIX AND NOT APPLE)
  find_library(UDEV_LIBRARY udev)
  if(UDEV_LIBRARY)
    target_link_libraries(autodriver PRIVATE ${UDEV_LIBRARY})
    target_compile_definitions(autodriver PRIVATE AUTODRIVER_HAVE_UDEV)
  endif()
endif()

if(AUTODRIVER_WITH_ORBBEC)
  find_package(OrbbecSDK QUIET)
  if(OrbbecSDK_FOUND OR TARGET OrbbecSDK::OrbbecSDK)
    message(STATUS "autodriver: OrbbecSDK enabled")
    if(TARGET OrbbecSDK::OrbbecSDK)
      target_link_libraries(autodriver PUBLIC OrbbecSDK::OrbbecSDK)
    else()
      target_include_directories(autodriver PUBLIC ${OrbbecSDK_INCLUDE_DIRS})
      target_link_libraries(autodriver PUBLIC ${OrbbecSDK_LIBRARY})
    endif()
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_ORBBEC)
  else()
    message(STATUS "autodriver: OrbbecSDK not found; Orbbec backends stubbed")
  endif()
endif()

if(AUTODRIVER_WITH_REALSENSE)
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    message(STATUS "autodriver: librealsense2 ${realsense2_VERSION} enabled")
    target_sources(autodriver PRIVATE
      camera/realsense/imu_driver.cpp
      camera/realsense/camera_driver.cpp
      camera/realsense/pointcloud_driver.cpp)
    target_link_libraries(autodriver PUBLIC realsense2::realsense2)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_REALSENSE)
  else()
    message(STATUS
      "autodriver: librealsense2 not found; RealSense drivers stubbed "
      "(ParseStreamKind / MatchesModelFilter still available)")
  endif()
endif()

if(AUTODRIVER_WITH_RPLIDAR)
  find_package(RplidarSDK QUIET)
  if(RplidarSDK_FOUND)
    message(STATUS
      "autodriver: RPLidar SDK enabled (${RplidarSDK_INCLUDE_DIR}, "
      "${RplidarSDK_LIBRARY})")
    target_link_libraries(autodriver PRIVATE RplidarSDK::RplidarSDK)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_RPLIDAR)
  else()
    message(STATUS
      "autodriver: RplidarSDK not found; install via "
      "scripts/install_rplidar_sdk.sh or set RplidarSDK_ROOT "
      "(RPLidar backend stubs to nullptr)")
  endif()
endif()

if(AUTODRIVER_WITH_LIVOX)
  find_package(LivoxSDK2 QUIET)
  if(LivoxSDK2_FOUND)
    message(STATUS
      "autodriver: Livox-SDK2 enabled (${LivoxSDK2_INCLUDE_DIR})")
    target_link_libraries(autodriver PRIVATE LivoxSDK2::LivoxSDK2)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_LIVOX_SDK2)
  else()
    message(STATUS
      "autodriver: Livox-SDK2 not found; install via "
      "scripts/install_livox_sdk2.sh (Mid-360/HAP/Avia2)")
  endif()
  find_package(LivoxSDK QUIET)
  if(LivoxSDK_FOUND)
    message(STATUS
      "autodriver: Livox-SDK enabled (${LivoxSDK_INCLUDE_DIR})")
    target_link_libraries(autodriver PRIVATE LivoxSDK::LivoxSDK)
    target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_LIVOX_SDK1)
  else()
    message(STATUS
      "autodriver: Livox-SDK (v1) not found; install via "
      "scripts/install_livox_sdk.sh (Mid-40/70/Horizon/Avia)")
  endif()
endif()

