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

cmake_minimum_required(VERSION 3.14)

project(autoviz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AUTOVIZ_BUILD_TEST "Build unit tests" OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
# Prefer local find modules under autoviz/cmake.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Websocketpp REQUIRED)
find_package(nlohmann_json QUIET)
find_package(Protobuf REQUIRED)

if(NOT TARGET autolink)
  find_package(autolink REQUIRED)
endif()

# automsgs: optional, for message schema / types if needed
find_package(automsgs QUIET)

# -----------------------------------------------------------------------------
# version.cpp
# -----------------------------------------------------------------------------
find_program(GIT_EXECUTABLE git)
if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE AUTOVIZ_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
if(NOT AUTOVIZ_GIT_HASH)
  set(AUTOVIZ_GIT_HASH "unknown")
endif()
set(FOXGLOVE_BRIDGE_VERSION "${CMAKE_PROJECT_VERSION}")
set(FOXGLOVE_BRIDGE_GIT_HASH "${AUTOVIZ_GIT_HASH}")
configure_file(
  core/common/version.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/core/common/version.cpp
  @ONLY)

# -----------------------------------------------------------------------------
# foxglove_bridge_base (WebSocket protocol, same as ros-foxglove-bridge base)
# -----------------------------------------------------------------------------
add_library(foxglove_bridge_base STATIC
  core/common/base64.cpp
  core/common/parameter.cpp
  core/common/serialization.cpp
  core/common/server_factory.cpp
  core/common/foxglove_bridge.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/core/common/version.cpp
)
target_include_directories(foxglove_bridge_base PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/core/common
)
target_link_libraries(foxglove_bridge_base PUBLIC
  OpenSSL::SSL
  OpenSSL::Crypto
  ZLIB::ZLIB
  Threads::Threads
)
if(nlohmann_json_FOUND)
  target_link_libraries(foxglove_bridge_base PUBLIC nlohmann_json::nlohmann_json)
endif()

# -----------------------------------------------------------------------------
# autoviz executable (autolink bridge + main)
# -----------------------------------------------------------------------------
add_executable(autoviz
  core/main.cpp
  core/bridge/autolink_bridge.cpp
)
target_include_directories(autoviz PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/autolink
  ${CMAKE_CURRENT_SOURCE_DIR}/core/common
  ${CMAKE_CURRENT_SOURCE_DIR}/core/bridge
)
target_link_libraries(autoviz PRIVATE
  foxglove_bridge_base
  autolink
)
if(automsgs_FOUND AND TARGET automsgs)
  target_link_libraries(autoviz PRIVATE automsgs)
  target_include_directories(autoviz PRIVATE ${AUTOMSGS_INCLUDE_DIRS})
endif()

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS autoviz RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# -----------------------------------------------------------------------------
# Tests (optional)
# -----------------------------------------------------------------------------
if(AUTOVIZ_BUILD_TEST)
  file(GLOB AUTOVIZ_TESTS "core/common/test/*_test.cpp")
  foreach(TEST_SRC ${AUTOVIZ_TESTS})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_include_directories(${TEST_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/core/common
      ${CMAKE_CURRENT_SOURCE_DIR}/core/common/test
    )
    target_link_libraries(${TEST_NAME} PRIVATE foxglove_bridge_base)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  endforeach()
endif()
