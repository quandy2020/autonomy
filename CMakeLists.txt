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

# Read version from version.json using common function
include("${PROJECT_SOURCE_DIR}/cmake/version.cmake")
read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")

set(AUTONOMY_HAS_GRPC ${BUILD_GRPC})
option(BUILD_GRPC "Build autonomy gRPC support" true)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" false)
option(BUILD_DOCS "Build autonomy docs support" true)
option(BUILD_TEST "Build autonomy unit test" true)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_autonomy_project()
google_enable_testing()

# Set module path to find custom Find modules
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

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
find_package(Adolc REQUIRED)
find_package(Ipopt QUIET)
find_package(FBow REQUIRED)
find_package(G2o REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)

option(BUILD_ONNXRUNTIME "Build ONNX Runtime support" true)
find_package(OnnxRuntime QUIET)

if (${BUILD_GRPC})
  find_package(gRPC REQUIRED)
endif()

# Only build the documentation if we can find Sphinx.
if (${BUILD_DOCS})
  find_package(Sphinx)
  if(SPHINX_FOUND)
    add_subdirectory("docs")
  endif()
endif()


# autolink (embedded build: skip autolink tests/examples to speed up colcon)
set(AUTOLINK_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(AUTOLINK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(AUTOLINK_BUILD_TOOLS ON CACHE BOOL "" FORCE)
set(AUTOLINK_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(AUTOLINK_BUILD_DOCS OFF CACHE BOOL "" FORCE)
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/autolink/CMakeLists.txt")
  message(FATAL_ERROR
    "autolink submodule is not initialized.\n"
    "From the repository root, run:\n"
    "  git submodule update --init --recursive autolink")
endif()
add_subdirectory(autolink)

set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/config
  CACHE PATH ".lua configuration files directory")
set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/config
  CACHE PATH ".lua configuration files directory for cartographer")

install(DIRECTORY config DESTINATION share/autonomy)
install(DIRECTORY cmake DESTINATION share/autonomy/)

include("${PROJECT_SOURCE_DIR}/cmake/autonomy_sources.cmake")

# Remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
endif()

list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})

# BT plugins are built as separate .so files (see cmake/plugin/behavior_tree_plugins.cmake).
file(GLOB_RECURSE BEHAVIOR_TREE_PLUGIN_SRCS
  "autonomy/navigator/behavior_tree/plugins/*.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${BEHAVIOR_TREE_PLUGIN_SRCS})
file(GLOB_RECURSE TASK_BEHAVIOR_TREE_PLUGIN_SRCS
  "autonomy/task/apps/*/plugins/*/*.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TASK_BEHAVIOR_TREE_PLUGIN_SRCS})
file(GLOB_RECURSE VISUALIZATION_SRCS
  "autonomy/visualization/*.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${VISUALIZATION_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TESTS})

# Exclude ONNX Runtime network wrapper when not building with ONNX
file(GLOB_RECURSE ONNX_NETWORK_SRCS "autonomy/common/network/*.cpp")
if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND)
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ONNX_NETWORK_SRCS})
endif()

if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_HDRS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_SRCS})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_GRPC_HDRS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_GRPC_SRCS})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_BRIDGE_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_BRIDGE_FILES})
endif()

if (NOT ${BUILD_TOOLS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_TOOLS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TOOLS})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_TOOLS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TOOLS})
  list(REMOVE_ITEM ALL_TESTS ${ALL_TOOLS})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_TOOLS})
endif()

file(GLOB_RECURSE ALL_PROTOS "autonomy/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "autonomy/*_service.proto")
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_PROTOS ALL_GRPC_SERVICES)
endif()

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

set(PROTOBUF_PROTOC_EXECUTABLE "${Protobuf_PROTOC_EXECUTABLE}")

if(${BUILD_GRPC})
  find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
  if(NOT GRPC_CPP_PLUGIN)
    message(FATAL_ERROR "grpc_cpp_plugin not found. Please install gRPC and make sure it's in your PATH.")
  endif()
  set(ALL_GRPC_SERVICE_SRCS)
  set(ALL_GRPC_SERVICE_HDRS)

  message("${ALL_GRPC_SERVICES}")
  foreach(ABS_FIL ${ALL_GRPC_SERVICES})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h")
    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --grpc_out=${PROJECT_BINARY_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --cpp_out  ${PROJECT_BINARY_DIR}
        -I ${PROJECT_SOURCE_DIR}
        ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
      VERBATIM
    )
  endforeach()
  set_source_files_properties(${ALL_GRPC_SERVICE_SRCS} ${ALL_GRPC_SERVICE_HDRS} PROPERTIES GENERATED TRUE)
  list(APPEND ALL_LIBRARY_HDRS ${ALL_GRPC_SERVICE_HDRS})
  list(APPEND ALL_LIBRARY_SRCS ${ALL_GRPC_SERVICE_SRCS})
endif()


foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})
set(INSTALL_GENERATED_HDRS ${ALL_PROTO_HDRS} ${ALL_GRPC_SERVICE_HDRS})

# configuration files
configure_file(
  ${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp.cmake
  ${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp)

# autonomy version
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/autonomy-config-version.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake"
      DESTINATION "share/autonomy")

# Generate source file with version definitions (build tree only; not committed).
include("${PROJECT_SOURCE_DIR}/cmake/generate_version_definitions.cmake")
list(REMOVE_ITEM ALL_LIBRARY_SRCS
  "${PROJECT_SOURCE_DIR}/autonomy/common/version.cpp")
list(APPEND ALL_LIBRARY_SRCS ${AUTONOMY_VERSION_CPP})
set_source_files_properties(${AUTONOMY_VERSION_CPP} PROPERTIES GENERATED TRUE)

## Define Eigen addons. 
include(${PROJECT_SOURCE_DIR}/cmake/grid_map_core-extras.cmake)

# Task scheduler uses Taskflow (C++20); built as a separate object library.
set(AUTONOMY_TASK_SCHEDULER_SRCS
  "${PROJECT_SOURCE_DIR}/autonomy/task/scheduler/scheduler.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${AUTONOMY_TASK_SCHEDULER_SRCS})

# Autonomy library
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

# protobuf
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Protobuf_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf)

# Eigen3
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

# Ceres solver
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

# Lua5.3
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

# OpenCV
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

# Yaml-cpp
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

# OSQP
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OSQP_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OSQP_LIBRARIES})

# glog
target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

# FBow
target_link_libraries(${PROJECT_NAME} PUBLIC FBow::fbow)

# g2o (atlas optimization)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_sim3
    g2o::solver_dense
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
)

# SQLite3 (atlas map database)
target_link_libraries(${PROJECT_NAME} PUBLIC ${SQLite3_LIBRARIES})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${SQLite3_INCLUDE_DIRS})

# autolink (planning plugin_manager)
target_link_libraries(${PROJECT_NAME} PUBLIC autolink)

# Cartographer (pbstream gzip I/O, submap painting)
target_link_libraries(${PROJECT_NAME} PUBLIC Boost::iostreams PkgConfig::CAIRO)

include("${PROJECT_SOURCE_DIR}/cmake/plugin/behavior_tree_plugins.cmake")
autonomy_link_behaviortree_cpp(${PROJECT_NAME} PUBLIC)
autonomy_add_behavior_tree_plugins(${PROJECT_NAME})

include("${PROJECT_SOURCE_DIR}/cmake/plugin/task_behavior_tree_plugins.cmake")
autonomy_add_task_behavior_tree_plugins(${PROJECT_NAME})

set(AUTONOMY_PROTO_GENERATED_HDRS ${ALL_PROTO_HDRS})
if(BUILD_GRPC)
  list(APPEND AUTONOMY_PROTO_GENERATED_HDRS ${ALL_GRPC_SERVICE_HDRS})
endif()

include("${PROJECT_SOURCE_DIR}/cmake/taskflow.cmake")
autonomy_link_taskflow(${PROJECT_NAME} PUBLIC)

if(${BUILD_GRPC})
  google_binary(autonomy_bridge
    SRCS
      autonomy/bridge/bridge_main.cpp
  )

  install(
    DIRECTORY autonomy/bridge/launch/
    DESTINATION share/autonomy/bridge/launch
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING
    PATTERN "*.launch"
  )
endif()

google_binary(localization
  SRCS
    autonomy/localization/localization_main.cpp
)

google_binary(planning
  SRCS
    autonomy/planning/planning_main.cpp
)

google_binary(control
  SRCS
    autonomy/control/control_main.cpp
)

google_binary(task
  SRCS
    autonomy/task/task_main.cpp
)

install(
  DIRECTORY autonomy/planning/launch/
  DESTINATION share/autonomy/planning/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.launch"
)

install(
  DIRECTORY autonomy/control/launch/
  DESTINATION share/autonomy/control/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.launch"
)

install(
  DIRECTORY autonomy/task/launch/
  DESTINATION share/autonomy/task/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.launch"
)

google_binary(cartographer_occupancy_grid_node
  SRCS
    autonomy/localization/cartographer/node/occupancy_grid_node_main.cpp
)

google_binary(cartographer_pbstream_to_map
  SRCS
    autonomy/localization/cartographer/node/pbstream_to_map_main.cpp
)

find_package(foxglove-sdk CONFIG QUIET)

google_binary(autonomy_foxglove_fakedata
  SRCS
    autonomy/visualization/fakedata_test.cpp
)

if(foxglove-sdk_FOUND)
  find_package(ZLIB REQUIRED)
  google_binary(autonomy_foxglove_bridge
    SRCS
      autonomy/visualization/foxglove_main.cpp
      autonomy/visualization/transport/autolink_discovery.cpp
      autonomy/visualization/visualization_server.cpp
      autonomy/visualization/transport/channel_forwarder.cpp
      autonomy/visualization/adapters/message_adapter.cpp
      autonomy/visualization/adapters/foxglove_converters.cpp
      autonomy/visualization/schema/schema_compatibility.cpp
      autonomy/visualization/common/visualization_schema_registry.cpp
      autonomy/visualization/transport/foxglove_bridge_server.cpp
  )
  target_link_libraries(autonomy_foxglove_bridge
    PRIVATE
      foxglove-sdk::foxglove_cpp_shared
      ZLIB::ZLIB
  )
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(autonomy_foxglove_bridge PRIVATE
      -Wno-error=maybe-uninitialized)
  endif()
else()
  message(WARNING
    "foxglove-sdk not found, skipping autonomy_foxglove_bridge. "
    "Set CMAKE_PREFIX_PATH or install foxglove-sdk first.")
endif()

install(
  DIRECTORY autonomy/localization/launch/
  DESTINATION share/autonomy/localization/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.launch"
)

if(BUILD_PROMETHEUS)
  find_package(prometheus-cpp CONFIG REQUIRED)
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PROMETHEUS=1)
  target_link_libraries(${PROJECT_NAME} PRIVATE prometheus-cpp-core prometheus-cpp-pull)
endif()

if(${BUILD_GRPC})
  target_link_libraries(${PROJECT_NAME} PUBLIC grpc++ grpc)
endif()

# Ipopt (Interior Point Optimizer)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${IPOPT_LIBRARIES})

# ONNX Runtime (optional, common/network)
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${OnnxRuntime_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${OnnxRuntime_LIBRARIES})
endif()

# fbow/g2o are often installed under /usr/local/lib; embed their paths so
# libautonomy.so resolves NEEDED libs at runtime without LD_LIBRARY_PATH.
function(_autonomy_append_rpath_from_target _rpath_var _target)
  if(NOT TARGET ${_target})
    return()
  endif()
  get_target_property(_loc ${_target} IMPORTED_LOCATION)
  if(NOT _loc)
    get_target_property(_loc ${_target} IMPORTED_LOCATION_RELEASE)
  endif()
  if(_loc)
    get_filename_component(_dir "${_loc}" DIRECTORY)
    list(APPEND ${_rpath_var} "${_dir}")
    set(${_rpath_var} "${${_rpath_var}}" PARENT_SCOPE)
  endif()
endfunction()

set(_autonomy_install_rpath "")
_autonomy_append_rpath_from_target(_autonomy_install_rpath FBow::fbow)
_autonomy_append_rpath_from_target(_autonomy_install_rpath g2o::core)
list(REMOVE_DUPLICATES _autonomy_install_rpath)
if(_autonomy_install_rpath)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${_autonomy_install_rpath}"
    INSTALL_RPATH "${_autonomy_install_rpath}"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

## Test unit
if (${BUILD_TEST})
  set(TEST_LIB autonomy_test_library)
  add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
  target_include_directories(${TEST_LIB} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY})
  target_link_libraries(${TEST_LIB} PUBLIC ${PROJECT_NAME})
  
  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    # Replace slashes as required for CMP0037.
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    # Ensure libautonomy is fully linked before unit tests (avoids undefined refs
    # when incremental builds relink tests against a stale/incomplete shared lib).
    add_dependencies("${TEST_TARGET_NAME}" ${PROJECT_NAME})
    if(${BUILD_GRPC})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++ grpc)
    endif()
    if(${BUILD_PROMETHEUS})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-core)
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-pull)
    endif()
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
  endforeach()
endif()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/autolink>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# autolink is installed with EXPORT AutonomyExport from autolink/CMakeLists.txt
# when embedded in this project; do not list it here again.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT AutonomyExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

file(GLOB_RECURSE INTERNAL_HDRS "autonomy/*/internal/*.h" "autonomy/*/internal/*.hpp")
file(GLOB_RECURSE INSTALL_LEGACY_HDRS "autonomy/*.h")

set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS} ${INSTALL_LEGACY_HDRS})
# Generated .pb.h / .grpc.pb.h live under ${PROJECT_BINARY_DIR}/autonomy/...; if
# installed via PROJECT_SOURCE_DIR-relative paths they end up as
# include/build/autonomy/... (duplicate of the INSTALL_GENERATED_HDRS loop below).
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${ALL_PROTO_HDRS})
if(BUILD_GRPC)
  list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${ALL_GRPC_SERVICE_HDRS})
endif()
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})
if(ALL_DOTFILES)
  list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${ALL_DOTFILES})
endif()
if(NOT ${BUILD_GRPC})
  list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${ALL_GRPC_HDRS})
endif()

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

set(AUTONOMY_CMAKE_DIR share/autonomy/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  autonomy-config.cmake.in
  ${PROJECT_BINARY_DIR}/autonomy-config.cmake
  PATH_VARS AUTONOMY_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/autonomy
)

# Install the export set for find_package
install(
  EXPORT AutonomyExport
  NAMESPACE autonomy::
  DESTINATION share/autonomy/cmake
  FILE AutonomyTargets.cmake
  EXPORT_LINK_INTERFACE_LIBRARIES
)

install(
  FILES ${PROJECT_BINARY_DIR}/autonomy-config.cmake
  DESTINATION share/autonomy/
)