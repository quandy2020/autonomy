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

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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
option(BUILD_GRID_MAP_DEMOS "Build ROS-free grid_map demos" ON)
option(BUILD_AUTODRIVER "Build autodriver sensor HAL subproject" ON)
option(BUILD_AUTOVIZ "Build native 3D visualization tool (autoviz)" ON)
option(BUILD_AUTOSIM "Build autonomous simulation tool (autosim)" ON)
set(AUTONOMY_HAS_GRPC ${BUILD_GRPC})

option(BUILD_ONNXRUNTIME "Build ONNX Runtime inference support" ON)

option(AUTONOMY_EXPLORATION_ORTOOLS
  "Link vendored OR-Tools for exploration TSP (set AUTONOMY_ORTOOLS_ROOT)" OFF)
set(AUTONOMY_ORTOOLS_ROOT "" CACHE PATH
  "Root of vendored or-tools (include/ + lib/libortools.so)")

if(NOT AUTONOMY_ORTOOLS_ROOT)
  set(_ORTOOLS_CANDIDATES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../ros2/autonomous_exploration_development_environment/src/tare_planner/src/tare_planner/or-tools"
    "/home/quandy/workspace/github/ros2/autonomous_exploration_development_environment/src/tare_planner/src/tare_planner/or-tools")
  foreach(_cand IN LISTS _ORTOOLS_CANDIDATES)
    if(EXISTS "${_cand}/lib/libortools.so")
      set(AUTONOMY_ORTOOLS_ROOT "${_cand}" CACHE PATH
        "Auto-detected TARE vendored or-tools" FORCE)
      set(AUTONOMY_EXPLORATION_ORTOOLS ON CACHE BOOL
        "Auto-enable OR-Tools from TARE vendored path" FORCE)
      message(STATUS "Exploration OR-Tools: ${_cand}")
      break()
    endif()
  endforeach()
endif()

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
# Autoviz needs Qt6 versionless Qt::* targets. Find Qt6 before OpenCV/PCL so
# their Qt5 transitive deps (highgui / VTK GUISupportQt) do not claim Qt::Core
# first and break Qt6CoreVersionlessTargets.
if(BUILD_AUTOVIZ)
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGLWidgets OpenGL Xml Svg Network)
endif()
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
find_package(PCL REQUIRED COMPONENTS
  common
  features
  filters
  io
  kdtree
  segmentation
  surface)
find_package(OpenMP QUIET)
find_package(TBB REQUIRED)

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

set(_autonomy_config_dir ${CMAKE_INSTALL_PREFIX}/share/autonomy/config)
set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY ${_autonomy_config_dir}
  CACHE PATH ".lua configuration files directory")
set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${_autonomy_config_dir}
  CACHE PATH ".lua configuration files directory for cartographer")
unset(_autonomy_config_dir)

# -----------------------------------------------------------------------------
# 4. Sources + codegen
# -----------------------------------------------------------------------------
if(AUTONOMY_EXPLORATION_ORTOOLS)
  find_package(ORtools)
endif()

include("${PROJECT_SOURCE_DIR}/cmake/autonomy_sources.cmake")
autonomy_filter_library_sources()

add_subdirectory(automsgs)

if(BUILD_AUTODRIVER)
  include("${PROJECT_SOURCE_DIR}/autodriver/cmake/autodriver_embed.cmake")
endif()

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

# Drop truncated .so files before linking (interrupted builds leave 0-byte libs).
add_custom_target(${PROJECT_NAME}_scrub_shared_libs ALL
  COMMAND ${CMAKE_COMMAND}
    -Dlib_dir=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    -P ${PROJECT_SOURCE_DIR}/cmake/scrub_autonomy_shared_libraries.cmake
  COMMENT "Checking shared libraries for corruption")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_scrub_shared_libs)
if(TARGET autolink)
  add_dependencies(autolink ${PROJECT_NAME}_scrub_shared_libs)
endif()
if(TARGET automsgs)
  add_dependencies(automsgs ${PROJECT_NAME}_scrub_shared_libs)
endif()
if(TARGET automsgs_proto_copy)
  add_dependencies(${PROJECT_NAME} automsgs_proto_copy)
endif()
include("${PROJECT_SOURCE_DIR}/cmake/autonomy_deps.cmake")
autonomy_link_dependencies(${PROJECT_NAME})

# Fathom's autolink component requires the concrete ONNX/common-network
# runner and is deliberately a separate mainboard-loadable shared library.
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  add_subdirectory(autonomy/perception/fathom)
endif()

# Echo tool needs automsgs protos for DebugString when topology omits type.
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
google_binary(autonomy.localization
  SRCS autonomy/localization/localization_main.cpp)
google_binary(autonomy.planning
  SRCS autonomy/planning/planning_main.cpp)
google_binary(autonomy.control
  SRCS autonomy/control/control_main.cpp)
google_binary(autonomy.task
  SRCS autonomy/task/task_main.cpp)
google_binary(autonomy.perception
  SRCS autonomy/perception/perception_main.cpp)
if(foxglove-sdk_FOUND)
  # google_binary(autonomy_foxglove_fakedata
  #   SRCS autonomy/visualization/fakedata_test.cpp)
  google_binary(autonomy_foxglove_bridge
    SRCS autonomy/visualization/foxglove_main.cpp)
endif()

if(BUILD_GRID_MAP_DEMOS)
  add_subdirectory(autonomy/map/grid_map/grid_map_demos)
endif()

if(BUILD_AUTOVIZ)
  set(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY ON)
  add_subdirectory(autoviz)
  unset(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY)
endif()

if(BUILD_AUTOSIM)
  include("${PROJECT_SOURCE_DIR}/cmake/autosim_embed.cmake")
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
