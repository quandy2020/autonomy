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

cmake_minimum_required(VERSION 3.20)

project(autodriver LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AUTODRIVER_BUILD_TEST "Build unit tests" ON)
option(AUTODRIVER_BUILD_EXAMPLES "Build example binaries" ON)
option(AUTODRIVER_BUILD_AUTONOMY_BRIDGE "Build autonomy integration bridge" OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(AUTODRIVER_WITH_REALSENSE "Build Intel RealSense camera/IMU drivers" ON)

set(AUTODRIVER_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(autodriver)

if(AUTODRIVER_BUILD_AUTONOMY_BRIDGE)
  if(TARGET autonomy)
    add_subdirectory(bridge/autonomy)
  else()
    message(WARNING "autodriver autonomy bridge skipped: autonomy target not found")
  endif()
endif()

if(AUTODRIVER_BUILD_EXAMPLES)
  add_executable(autodriver_demo examples/demo_main.cpp)
  target_link_libraries(autodriver_demo PRIVATE autodriver)
  target_include_directories(autodriver_demo PRIVATE
    ${AUTODRIVER_ROOT_DIR}/autodriver)
endif()

if(AUTODRIVER_BUILD_TEST)
  enable_testing()
  find_package(GTest REQUIRED)

  function(autodriver_add_test NAME SRC)
    add_executable(${NAME} ${SRC})
    target_link_libraries(${NAME} PRIVATE autodriver GTest::gtest_main)
    target_include_directories(${NAME} PRIVATE
      ${AUTODRIVER_ROOT_DIR}/autodriver)
    add_test(NAME ${NAME} COMMAND ${NAME})
  endfunction()

  autodriver_add_test(test_time_sync test/test_time_sync.cpp)
  autodriver_add_test(test_sensor_hub test/test_sensor_hub.cpp)
  autodriver_add_test(test_sensor_manager test/test_sensor_manager.cpp)
  autodriver_add_test(test_nmea0183 test/test_nmea0183.cpp)
  autodriver_add_test(test_wit_motion test/test_wit_motion.cpp)
  autodriver_add_test(test_realsense_types test/test_realsense_types.cpp)
  autodriver_add_test(test_autodriver_config test/test_autodriver_config.cpp)
endif()

include(GNUInstallDirs)
include(${AUTODRIVER_ROOT_DIR}/cmake/autodriver_install.cmake)
