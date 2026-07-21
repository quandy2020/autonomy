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

cmake_minimum_required(VERSION 3.20)

if(POLICY CMP0177)
  cmake_policy(SET CMP0177 NEW)
endif()

project(autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------------------------------------------------------
# 1. Options
# -----------------------------------------------------------------------------
include("${PROJECT_SOURCE_DIR}/cmake/version.cmake")
read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")

option(BUILD_GRPC "Build autonomy gRPC support" ON)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" OFF)
option(BUILD_DOCS "Build autonomy docs support" ON)
option(BUILD_TEST "Build autonomy unit test" ON)
option(BUILD_TOOLS "Build autonomy tools sources" ON)
option(BUILD_AUTODRIVER "Build autodriver sensor HAL subproject" OFF)
set(AUTONOMY_HAS_GRPC ${BUILD_GRPC})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_autonomy_project()
google_enable_testing()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

# -----------------------------------------------------------------------------
# 2. Dependencies
# -----------------------------------------------------------------------------
# Required
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(glog REQUIRED)
find_package(OSQP REQUIRED)
find_package(OpenCV REQUIRED)
include(EnsureProtobuf319)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(FBow REQUIRED)
find_package(G2o REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)

# Optional
find_package(Ipopt QUIET)
find_package(foxglove-sdk CONFIG QUIET)
find_package(OnnxRuntime QUIET)
if(BUILD_GRPC)
  find_package(gRPC REQUIRED)
endif()
if(BUILD_PROMETHEUS)
  find_package(prometheus-cpp CONFIG REQUIRED)
endif()
if(foxglove-sdk_FOUND)
  find_package(ZLIB REQUIRED)
endif()

# -----------------------------------------------------------------------------
# 3. Subprojects (docs / autolink)
# -----------------------------------------------------------------------------
if(BUILD_DOCS)
  find_package(Sphinx)
  if(SPHINX_FOUND)
    add_subdirectory("docs")
  endif()
endif()

include("${PROJECT_SOURCE_DIR}/cmake/autolink_embed.cmake")

if(BUILD_AUTODRIVER)
  include("${PROJECT_SOURCE_DIR}/autodriver/cmake/autodriver_embed.cmake")
endif()

set(_autonomy_config_dir ${CMAKE_INSTALL_PREFIX}/share/autonomy/config)
set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY ${_autonomy_config_dir}
  CACHE PATH ".lua configuration files directory")
set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${_autonomy_config_dir}
  CACHE PATH ".lua configuration files directory for cartographer")
unset(_autonomy_config_dir)

# -----------------------------------------------------------------------------
# 4. Sources + codegen
# -----------------------------------------------------------------------------
include("${PROJECT_SOURCE_DIR}/cmake/autonomy_sources.cmake")
autonomy_filter_library_sources()

include("${PROJECT_SOURCE_DIR}/cmake/protobuf.cmake")

configure_file(
  ${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp.cmake
  ${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/autonomy-config-version.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake @ONLY)

include("${PROJECT_SOURCE_DIR}/cmake/generate_version_definitions.cmake")
list(REMOVE_ITEM ALL_LIBRARY_SRCS
  "${PROJECT_SOURCE_DIR}/autonomy/common/version.cpp")
list(APPEND ALL_LIBRARY_SRCS ${AUTONOMY_VERSION_CPP})
set_source_files_properties(${AUTONOMY_VERSION_CPP} PROPERTIES GENERATED TRUE)

include(${PROJECT_SOURCE_DIR}/cmake/grid_map_core-extras.cmake)

# -----------------------------------------------------------------------------
# 5. libautonomy + link
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
include("${PROJECT_SOURCE_DIR}/cmake/autonomy_deps.cmake")
autonomy_link_dependencies(${PROJECT_NAME})

if(BUILD_AUTODRIVER AND TARGET autodriver)
  add_subdirectory(autodriver/bridge/autonomy)

  google_binary(autodriver_hub
    SRCS autodriver/examples/hub_main.cpp)
  target_link_libraries(autodriver_hub
    PRIVATE autodriver autodriver_autonomy_bridge)
  target_include_directories(autodriver_hub PRIVATE
    ${PROJECT_SOURCE_DIR}/autodriver/autodriver)
endif()

# Echo tool needs generated commsgs protos for DebugString when topology omits type.
if(TARGET autolink_channel)
  target_link_libraries(autolink_channel PRIVATE ${PROJECT_NAME})
  target_include_directories(autolink_channel PRIVATE
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR})
endif()

# -----------------------------------------------------------------------------
# 6. Binaries
# -----------------------------------------------------------------------------
if(BUILD_GRPC)
  google_binary(autonomy_bridge
    SRCS autonomy/bridge/bridge_main.cpp)
endif()
google_binary(localization
  SRCS autonomy/localization/localization_main.cpp)
google_binary(planning
  SRCS autonomy/planning/planning_main.cpp)
google_binary(control
  SRCS autonomy/control/control_main.cpp)
google_binary(exploration
  SRCS autonomy/exploration/exploration_main.cpp)
google_binary(task
  SRCS autonomy/task/task_main.cpp)
google_binary(monitor
  SRCS autonomy/system/monitor/monitor_main.cpp)
google_binary(task_teleop_smoke
  SRCS autonomy/task/tools/teleop_smoke_main.cpp)
google_binary(cartographer_occupancy
  SRCS autonomy/localization/cartographer/node/occupancy_grid_node_main.cpp)
google_binary(cartographer_pbstream_to_map
  SRCS autonomy/localization/cartographer/node/pbstream_to_map_main.cpp)
if(foxglove-sdk_FOUND)
  google_binary(autonomy_foxglove_fakedata
    SRCS autonomy/visualization/fakedata_test.cpp)
  google_binary(autonomy_foxglove_bridge
    SRCS autonomy/visualization/foxglove_main.cpp)
endif()

# -----------------------------------------------------------------------------
# 7. Tests
# -----------------------------------------------------------------------------
include("${PROJECT_SOURCE_DIR}/cmake/autonomy_tests.cmake")
autonomy_add_unit_tests()

# -----------------------------------------------------------------------------
# 8. Install / export
# -----------------------------------------------------------------------------
include("${PROJECT_SOURCE_DIR}/cmake/autonomy_install.cmake")
