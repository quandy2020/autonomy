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
  config/hub_config.cpp
  app/sensor_manager.cpp
  hal/sensor_factory.cpp
  io/serial_port.cpp
  io/can_socket.cpp
  io/realsense_device.cpp
  protocol/nmea0183.cpp
  protocol/wit_motion.cpp
  sync/time_sync.cpp
  sync/sample_buffer.cpp
  sync/sensor_hub.cpp
  drivers/driver_registry.cpp
  drivers/mock/mock_drivers.cpp
  drivers/hardware/driver_params.cpp
  drivers/hardware/realsense_types.cpp
  drivers/hardware/serial_gps_driver.cpp
  drivers/hardware/serial_imu_driver.cpp
  drivers/hardware/can_gps_driver.cpp
  drivers/hardware/can_imu_driver.cpp
  drivers/hardware/realsense_camera_driver.cpp
  drivers/hardware/realsense_imu_driver.cpp
  drivers/hardware/hardware_drivers.cpp
)

find_package(Threads REQUIRED)

if(AUTODRIVER_WITH_REALSENSE)
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    message(STATUS "autodriver: librealsense2 ${realsense2_VERSION} enabled")
  else()
    message(STATUS "autodriver: librealsense2 not found; RealSense drivers will fail at runtime")
  endif()
endif()

add_library(autodriver ${AUTODRIVER_SOURCES})

target_include_directories(autodriver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(autodriver PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(autodriver PUBLIC Threads::Threads)

if(AUTODRIVER_WITH_REALSENSE AND realsense2_FOUND)
  target_compile_definitions(autodriver PUBLIC AUTODRIVER_HAVE_REALSENSE)
  target_link_libraries(autodriver PUBLIC realsense2::realsense2)
endif()
