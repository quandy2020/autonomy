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
option(AUTOVIZ_BUILD_EXAMPLE "Build example" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
# Prefer local find modules under autoviz/cmake.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules")

find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Websocketpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Mcap QUIET)
find_package(nlohmann_json QUIET)
find_package(Protobuf REQUIRED)

# Reuse in-workspace targets when building as part of autonomy.
# Fallback to installed packages for standalone builds.
set(AUTOVIZ_AUTOLINK_TARGET "")
if(TARGET autolink)
  set(AUTOVIZ_AUTOLINK_TARGET autolink)
elseif(TARGET autolink::autolink)
  set(AUTOVIZ_AUTOLINK_TARGET autolink::autolink)
else()
  find_package(autolink REQUIRED)
  if(TARGET autolink)
    set(AUTOVIZ_AUTOLINK_TARGET autolink)
  elseif(TARGET autolink::autolink)
    set(AUTOVIZ_AUTOLINK_TARGET autolink::autolink)
  else()
    message(FATAL_ERROR "autolink target not found after find_package(autolink).")
  endif()
endif()

set(AUTOVIZ_AUTOMSGS_TARGET "")
if(TARGET automsgs)
  set(AUTOVIZ_AUTOMSGS_TARGET automsgs)
elseif(TARGET automsgs::automsgs)
  set(AUTOVIZ_AUTOMSGS_TARGET automsgs::automsgs)
else()
  find_package(automsgs REQUIRED)
  if(TARGET automsgs)
    set(AUTOVIZ_AUTOMSGS_TARGET automsgs)
  elseif(TARGET automsgs::automsgs)
    set(AUTOVIZ_AUTOMSGS_TARGET automsgs::automsgs)
  else()
    message(FATAL_ERROR "automsgs target not found after find_package(automsgs).")
  endif()
endif()

# -----------------------------------------------------------------------------
# autoviz core library and executable (autolink + automsgs only)
# -----------------------------------------------------------------------------
add_library(autoviz_core STATIC
  core/app.cpp
  core/config/config.cpp
  core/mcap/recorder.cpp
  core/server/foxglove_server.cpp
  core/bridge/autolink_bridge.cpp
  core/transport/auto_discovery.cpp
  core/converter/type_name_converter.cpp
  core/converter/schema_converter.cpp
)
target_include_directories(autoviz_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../autolink
  ${CMAKE_CURRENT_SOURCE_DIR}/core
)
target_link_libraries(autoviz_core PUBLIC
  ${AUTOVIZ_AUTOLINK_TARGET}
  ${AUTOVIZ_AUTOMSGS_TARGET}
  yaml-cpp
)
if(DEFINED AUTOMSGS_INCLUDE_DIRS)
  target_include_directories(autoviz_core PUBLIC ${AUTOMSGS_INCLUDE_DIRS})
endif()
if(MCAP_FOUND)
  # mcap writer.hpp only includes writer.inl when MCAP_IMPLEMENTATION is
  # defined (implementation is not fully header-only in this configuration).
  target_compile_definitions(autoviz_core PUBLIC AUTOVIZ_HAS_MCAP=1 MCAP_IMPLEMENTATION)
  target_include_directories(autoviz_core PUBLIC ${MCAP_INCLUDE_DIRS})
  if(MCAP_LIBRARIES)
    target_link_libraries(autoviz_core PUBLIC ${MCAP_LIBRARIES})
  endif()
endif()

add_executable(autoviz core/main.cpp)
target_link_libraries(autoviz PRIVATE autoviz_core)

# -----------------------------------------------------------------------------
# Example (optional)
# -----------------------------------------------------------------------------
if(AUTOVIZ_BUILD_EXAMPLE)
  add_subdirectory(example)
endif()

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS autoviz RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
if(AUTOVIZ_BUILD_EXAMPLE)
  install(TARGETS autoviz.example.datas_publisher RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# -----------------------------------------------------------------------------
# Tests (optional)
# -----------------------------------------------------------------------------
if(AUTOVIZ_BUILD_TEST)
  message(STATUS "AUTOVIZ_BUILD_TEST=ON, but tests are not migrated yet.")
endif()
