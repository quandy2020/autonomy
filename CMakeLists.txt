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

project(autonomy)

# Set CMake policies to suppress warnings
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)  # Prefer GLVND for OpenGL
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)  # Use uppercase <PACKAGENAME>_ROOT variables
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress warnings from third-party libraries being treated as errors
add_compile_options(-Wno-error=missing-braces -Wno-error=sign-compare)

# Suppress RPATH warnings for system directories  
if(POLICY CMP0060)
  cmake_policy(SET CMP0060 NEW)  # Link libraries by full path
endif()

if(POLICY CMP0095)
  cmake_policy(SET CMP0095 NEW)  # RPATH entries are properly escaped
endif()

# Read version from version.json using common function
include("${PROJECT_SOURCE_DIR}/cmake/version.cmake")
read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")

set(AUTONOMY_HAS_GRPC ${BUILD_GRPC})
option(BUILD_GRPC "Build autonomy gRPC support" true)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" false)
option(BUILD_DOCS "Build autonomy docs support" true)
option(BUILD_TEST "Build autonomy unit test" true)
option(BUILD_EXAMPLES "Build autonomy examples" true)
option(BUILD_FOXGLOVE "Build foxglove visualization support" true)
option(BUILD_TOOLS "Build autonomy tools" true)
option(BUILD_FORMAT_CODE "Build format code before building" true)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Ensure all targets (including plugins) can find libraries produced into
# the common lib/ directory at link time (e.g. vendored autolink).
link_directories(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_autonomy_project()
google_enable_testing()

# add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
#   add_link_options(-fsanitize=address)
  
# # Set a default install prefix if not specified by the user
# if(NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#   set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Default install prefix" FORCE)
# endif()

# Set module path to find custom Find modules
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

find_package(absl REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(glog REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(octomap REQUIRED)
find_package(OMPL REQUIRED)
find_package(OSQP REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(websocketpp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(FastDDS REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Adolc REQUIRED)
find_package(Ipopt QUIET)

option(BUILD_ONNXRUNTIME "Build ONNX Runtime inference support (common/network)" true)
find_package(OnnxRuntime QUIET)

# Create alias for behaviortree_cpp target if needed
if(TARGET BT::behaviortree_cpp AND NOT TARGET behaviortree_cpp::behaviortree_cpp)
    add_library(behaviortree_cpp::behaviortree_cpp ALIAS BT::behaviortree_cpp)
endif()

# Include Foxglove SDK vendor file (optional, controlled by BUILD_FOXGLOVE option)
if(${BUILD_FOXGLOVE})
  include("${PROJECT_SOURCE_DIR}/3rdparty/foxglove/autonomy_foxglove_vendor.cmake")
endif()

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

# Build autonomy visualizer (Qt QML application)
if (${BUILD_TOOLS})
  add_subdirectory("autonomy/tools")
endif()

set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/configuration_files
  CACHE PATH ".lua configuration files directory")
set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/configuration_files
  CACHE PATH ".lua configuration files directory for cartographer")

install(DIRECTORY configuration_files DESTINATION share/autonomy)
install(DIRECTORY cmake DESTINATION share/autonomy/)

# Include generate_plugins.cmake early to collect plugin files and create plugin libraries
# This must be done before GLOB_RECURSE so we can exclude plugin files
# This must also be done before examples so plugin libraries are available for linking
include(${PROJECT_SOURCE_DIR}/cmake/generate_plugins.cmake)

# Add automsgs and autolink subdirectories
add_subdirectory(automsgs)
add_subdirectory(autolink)


file(GLOB_RECURSE ALL_LIBRARY_HDRS "autonomy/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "autonomy/*.cpp")
file(GLOB_RECURSE AVIZ_FILES "autonomy/tools/aviz/*.cpp" "autonomy/tools/aviz/*.hpp" "autonomy/tools/aviz/*.h" "autonomy/tools/aviz/*.ui" "autonomy/tools/aviz/*.qrc")
file(GLOB_RECURSE ALL_GRPC_HDRS "autonomy/common/async_grpc/*.hpp" "autonomy/common/async_grpc/*.h" "autonomy/bridge/plugins/grpc/*.hpp")
file(GLOB_RECURSE ALL_GRPC_SRCS "autonomy/common/async_grpc/*.cpp" "autonomy/common/async_grpc/*.c" "autonomy/bridge/plugins/grpc/*.cpp")
file(GLOB_RECURSE ALL_GRPC_FILES "autonomy/common/async_grpc/*" "autonomy/bridge/plugins/grpc/*")
file(GLOB_RECURSE ALL_GRPC_BRIDGE_FILES "autonomy/bridge/bridge_server.*")
file(GLOB_RECURSE ALL_PROMETHEUS_FILES "autonomy/localization/cartographer/cloud/metrics/prometheus/*.hpp" "autonomy/localization/cartographer/cloud/metrics/prometheus/*.cpp")
file(GLOB_RECURSE TEST_LIBRARY_HDRS "autonomy/fake_*.hpp" "autonomy/*test_helpers*.hpp" "autonomy/*test*.hpp" "autonomy/mock_*.hpp")
file(GLOB_RECURSE TEST_LIBRARY_SRCS "autonomy/fake_*.cpp" "autonomy/*test_helpers*.cpp" "autonomy/*test*.cpp" "autonomy/mock_*.cpp")
file(GLOB_RECURSE ALL_TOOLS "autonomy/tools/*")
file(GLOB_RECURSE ALL_EXECUTABLES "autonomy/*main.cpp")
file(GLOB_RECURSE ALL_TESTS "autonomy/*_test.cpp")

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

# Remove plugin files (collected in generate_plugins.cmake, already in absolute paths)
if(ALL_PLUGINS_HDRS)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_PLUGINS_HDRS})
endif()
if(ALL_PLUGINS_SRCS)
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_PLUGINS_SRCS})
endif()
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TESTS})
# Exclude aviz files - they are built as a separate visualizer_lib target
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${AVIZ_FILES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${AVIZ_FILES})

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
if (NOT ${BUILD_PROMETHEUS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_PROMETHEUS_FILES})
endif()

# Foxglove visualization support (optional)
file(GLOB_RECURSE ALL_FOXGLOVE_FILES 
  "autonomy/visualization/*.hpp" 
  "autonomy/visualization/*.cpp" 
  "autonomy/visualization/*.h")
if (NOT BUILD_FOXGLOVE)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_FOXGLOVE_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_FOXGLOVE_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_FOXGLOVE_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_FOXGLOVE_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_FOXGLOVE_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_FOXGLOVE_FILES})
endif()

if (NOT ${BUILD_TOOLS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_TOOLS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TOOLS})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_TOOLS})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_TOOLS})
  list(REMOVE_ITEM ALL_TESTS ${ALL_TOOLS})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_TOOLS})
endif()

set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "autonomy/*/internal/*.h" "autonomy/*/internal/*.hpp")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})

# Create symlinks for control/controller/* to control/plugins/* during install
# This allows code to include from plugins/ directory while files are in controller/ directory
file(GLOB_RECURSE CONTROLLER_PLUGIN_HDRS "autonomy/control/controller/*/*.hpp" "autonomy/control/controller/*/*.h")
foreach(HDR ${CONTROLLER_PLUGIN_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  # Replace controller/ with plugins/ in the path
  string(REPLACE "control/controller/" "control/plugins/" PLUGIN_REL_FIL ${REL_FIL})
  get_filename_component(PLUGIN_DIR ${PLUGIN_REL_FIL} DIRECTORY)
  # Install to plugins directory instead of controller directory
  install(
    FILES ${HDR}
    DESTINATION include/${PLUGIN_DIR}
  )
endforeach()
# Remove controller headers from regular install list to avoid duplicate installation
list(FILTER INSTALL_SOURCE_HDRS EXCLUDE REGEX "autonomy/control/controller/")

file(GLOB_RECURSE ALL_PROTOS "autonomy/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "autonomy/*_service.proto")
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_PROTOS ALL_GRPC_SERVICES)
endif()

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

find_program(PROTOBUF_PROTOC_EXECUTABLE protoc)
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
  message(STATUS, "protoc not found. Please install protobuf and make sure it's in your PATH.")
endif()

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

# cartographer configuration files
configure_file(
  ${PROJECT_SOURCE_DIR}/autonomy/localization/cartographer/common/config.hpp.cmake
  ${PROJECT_SOURCE_DIR}/autonomy/localization/cartographer/common/config.hpp)

# autonomy version
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/autonomy-config-version.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/autonomy-config-version.cmake"
      DESTINATION "share/autonomy")

# Generate source file with version definitions.
include("${PROJECT_SOURCE_DIR}/cmake/generate_version_definitions.cmake")

## Define Eigen addons. 
include(${PROJECT_SOURCE_DIR}/cmake/grid_map_core-extras.cmake)

# 3rdparty
add_subdirectory(3rdparty/urdfdom_headers)
add_subdirectory(3rdparty/urdfdom)

# Autonomy library
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

# Auto-format code before building (if enabled)
google_format_code(
  TARGET_NAME ${PROJECT_NAME}
  ENABLED ${BUILD_FORMAT_CODE}
  PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}
)

# causes the shared library to have a version number
set_target_properties(${PROJECT_NAME} PROPERTIES 
  VERSION ${AUTONOMY_VERSION}
  SOVERSION ${AUTONOMY_MAJOR_VERSION}
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH FALSE
  INSTALL_RPATH "\$ORIGIN:${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_RPATH "\$ORIGIN:${CMAKE_BINARY_DIR}/lib"
  SKIP_BUILD_RPATH FALSE
)

# protobuf
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY})

# Eigen3
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

# Ceres solver
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})
# SuiteSparse (required by Ceres cholmod solver)
find_library(CHOLMOD_LIBRARY NAMES cholmod PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
if(CHOLMOD_LIBRARY)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${CHOLMOD_LIBRARY})
endif()
# LAPACK (required by Ceres)
find_package(LAPACK QUIET)
if(LAPACK_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LAPACK_LIBRARIES})
else()
  # Fallback: try to find lapack directly
  find_library(LAPACK_LIBRARY NAMES lapack PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
  if(LAPACK_LIBRARY)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${LAPACK_LIBRARY})
  endif()
endif()

# Lua5.3
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# Glog
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${GLOG_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARIES})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${PCL_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

# octomap
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OCTOMAP_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OCTOMAP_LIBRARIES})

# OpenCV
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

# Yaml-cpp
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

# graphicsmagick++
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${GRAPHICSMAGICKCPP_LIBRARIES})

# ompl
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OMPL_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OMPL_LIBRARIES})

# OSQP
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OSQP_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OSQP_LIBRARIES})

# glog
target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

# foxglove (optional)
if(${BUILD_FOXGLOVE})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.so)
  target_sources(${PROJECT_NAME} PRIVATE ${FOXGLOVE_SOURCES})
  target_compile_definitions(${PROJECT_NAME} PRIVATE AUTONOMY_HAS_FOXGLOVE)
  # Install foxglove library so executables can find it at runtime
  install(FILES ${foxglove_SOURCE_DIR}/lib/libfoxglove.so
    DESTINATION lib
    OPTIONAL
  )
endif()

# BehaviorTree.CPP
target_link_libraries(${PROJECT_NAME} PUBLIC behaviortree_cpp::behaviortree_cpp)

# prometheus (optional, for system/monitor and cartographer metrics/visualization)
if(${BUILD_PROMETHEUS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PROMETHEUS=1)
  find_package(prometheus-cpp CONFIG QUIET)
  if(prometheus-cpp_FOUND OR TARGET prometheus-cpp-core)
    target_link_libraries(${PROJECT_NAME} PRIVATE prometheus-cpp-core prometheus-cpp-pull)
  endif()
endif()

# abseil-cpp
set(ABSL_LIBRARY
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility)
target_link_libraries(${PROJECT_NAME} PUBLIC ${ABSL_LIBRARY})

if(${BUILD_GRPC})
  target_link_libraries(${PROJECT_NAME} PUBLIC grpc++ grpc)
  # if(${BUILD_PROMETHEUS})
  #   target_link_libraries(cartographer_grpc_server PUBLIC ${ZLIB_LIBRARIES})
  #   target_link_libraries(cartographer_grpc_server PUBLIC prometheus-cpp-core)
  #   target_link_libraries(cartographer_grpc_server PUBLIC prometheus-cpp-pull)
  # endif()
endif()

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${FastDDS_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  OpenSSL::Crypto
  OpenSSL::SSL
  ZLIB::ZLIB
  ${FastDDS_LIBRARIES})

# Cairo
find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED cairo)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${CAIRO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

# urdfdom_headers and urdfdom (for transform/urdf)
target_link_libraries(${PROJECT_NAME} PUBLIC urdfdom_headers urdf_parser)

# autolink
target_link_libraries(${PROJECT_NAME} PUBLIC autolink)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/autolink>)

# ADOL-C (Automatic Differentiation Library)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${ADOLC_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${ADOLC_LIBRARIES})

# Ipopt (Interior Point Optimizer)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${IPOPT_LIBRARIES})

# ONNX Runtime (optional, common/network)
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${OnnxRuntime_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${OnnxRuntime_LIBRARIES})
endif()

# autonomy.bridge.launcher
google_binary(autonomy.bridge.launcher
  SRCS
    autonomy/bridge/bridge_main.cpp
)

# autonomy.simulator.launcher
google_binary(autonomy.simulator.launcher
  SRCS
    autonomy/simulation/simulator_main.cpp
)

# autonomy.system.launcher
google_binary(autonomy.system.launcher
  SRCS
    autonomy/system/main.cpp
)

# autonomy.tasks.launcher (behavior-tree navigators, TaskManager)
google_binary(autonomy.tasks.launcher
  SRCS
    autonomy/tasks/main.cpp
)

# autonomy.tasks.navigator_client_test (action client for track_to_target testing; optional when not yet implemented)
if(EXISTS "${PROJECT_SOURCE_DIR}/autonomy/tasks/navigator/navigator_client_test.cpp")
  google_binary(autonomy.tasks.navigator_client_test
    SRCS
      autonomy/tasks/navigator/navigator_client_test.cpp
  )
endif()

# autonomy.visualization.launcher (requires foxglove)
if(${BUILD_FOXGLOVE})
  google_binary(autonomy.visualization.launcher
    SRCS
      autonomy/visualization/main.cpp
  )
endif()

# autonomy.visualization.data_publisher (publishes sensor data via autolink)
google_binary(autonomy.visualization.data_publisher
    SRCS
      autonomy/visualization/data_main.cpp
)

## Test unit
if (${BUILD_TEST})
  set(TEST_LIB autonomy_test_library)
  add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
  target_include_directories(${TEST_LIB} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY})
  target_link_libraries(${TEST_LIB} PUBLIC ${PROJECT_NAME})
  
  # Foxglove SDK for tests (if enabled)
  if(${BUILD_FOXGLOVE} AND FOXGLOVE_SDK_FOUND)
    if(FOXGLOVE_SDK_INCLUDE_DIRS)
      target_include_directories(${TEST_LIB} SYSTEM PRIVATE ${FOXGLOVE_SDK_INCLUDE_DIRS})
    endif()
    if(TARGET foxglove_sdk)
      target_link_libraries(${TEST_LIB} PRIVATE foxglove_sdk)
    endif()
  endif()

  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    # Replace slashes as required for CMP0037.
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    if(${BUILD_GRPC})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++ grpc)
    endif()
    if(${BUILD_PROMETHEUS})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${ZLIB_LIBRARIES})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-core)
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-pull)
    endif()
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
    
    # Foxglove SDK for individual test targets (if enabled)
    if(BUILD_FOXGLOVE AND FOXGLOVE_SDK_FOUND)
      if(FOXGLOVE_SDK_INCLUDE_DIRS)
        target_include_directories("${TEST_TARGET_NAME}" SYSTEM PRIVATE ${FOXGLOVE_SDK_INCLUDE_DIRS})
      endif()
      if(TARGET foxglove_sdk)
        target_link_libraries("${TEST_TARGET_NAME}" PRIVATE foxglove_sdk)
      endif()
    endif()
  endforeach()
endif()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Build examples (after plugins are created)
if (${BUILD_EXAMPLES})
  add_subdirectory("examples")
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT AutonomyExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# If we build vendored autolink from source, autonomy's exported library can
# link against `autolink` target. Add it to the export set so that
# `install(EXPORT ...)` generation doesn't fail.
if(TARGET autolink)
  install(
    TARGETS autolink
    EXPORT AutonomyExport
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
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
# Include autolink target in the export since autonomy depends on it
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

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/autonomy/launch 
  DESTINATION launch)