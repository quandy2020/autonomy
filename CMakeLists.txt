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

cmake_minimum_required(VERSION 3.5)

project(autonomy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(AUTONOMY_MAJOR_VERSION 0)
set(AUTONOMY_MINOR_VERSION 0)
set(AUTONOMY_PATCH_VERSION 1)
set(AUTONOMY_VERSION ${AUTONOMY_MAJOR_VERSION}.${AUTONOMY_MINOR_VERSION}.${AUTONOMY_PATCH_VERSION})
set(AUTONOMY_SOVERSION ${AUTONOMY_MAJOR_VERSION}.${AUTONOMY_MINOR_VERSION})

set(AUTONOMY_HAS_GRPC ${BUILD_GRPC})
option(BUILD_GRPC "Build autonomy gRPC support" false)
option(BUILD_CYBER "Build autonomy CyberRT support" false)
option(BUILD_DOCS "Build autonomy docs support" true)
option(BUILD_TOOLS "Build autonomy tools support" false)
option(BUILD_FOXGLOVE "Build autonomy visulization with foxglove support" false)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" false)
option(BUILD_TEST "Build autonomy unit test" false)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()

# # Set a default install prefix if not specified by the user
# if(NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#   set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Default install prefix" FORCE)
# endif()

find_package(absl REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OMPL REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)

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

# Use with CyberRT library
if (${BUILD_CYBER})
  find_package(CyberRT REQUIRED)
  find_package(FastRTPS REQUIRED)
endif()

set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/configuration_files
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/autonomy)
install(DIRECTORY cmake DESTINATION share/autonomy/)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "autonomy/*.hpp" "autonomy/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "autonomy/*.cpp")
file(GLOB_RECURSE TEST_LIBRARY_HDRS "autonomy/fake_*.hpp" "autonomy/*test_helpers*.hpp" "autonomy/*test*.hpp" "autonomy/mock_*.hpp")
file(GLOB_RECURSE TEST_LIBRARY_SRCS "autonomy/fake_*.cpp" "autonomy/*test_helpers*.cpp" "autonomy/*test*.cpp" "autonomy/mock_*.cpp")
file(GLOB_RECURSE ALL_TESTS "autonomy/*_test.cpp")
file(GLOB_RECURSE ALL_TOOLS_HRRS "autonomy/tools/*.hpp")
file(GLOB_RECURSE ALL_TOOLS_SRCS "autonomy/tools/*.cpp")
file(GLOB_RECURSE ALL_FOXGLOVE_HRRS "autonomy/visualization/*.hpp")
file(GLOB_RECURSE ALL_FOXGLOVE_SRCS "autonomy/visualization/*.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "autonomy/*_main.cpp")

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
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})

if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()
if (NOT ${BUILD_PROMETHEUS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_PROMETHEUS_FILES})
endif()
if (NOT ${BUILD_TOOLS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_TOOLS_HRRS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TOOLS_SRCS})
endif()
if (NOT ${BUILD_FOXGLOVE})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_FOXGLOVE_HRRS})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_FOXGLOVE_SRCS})
endif()


set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "cartographer/*/internal/*.h")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})

file(GLOB_RECURSE ALL_PROTOS "autonomy/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "autonomy/*_service.proto")
list(REMOVE_ITEM ALL_PROTOS ALL_GRPC_SERVICES)
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_PROTOS ${ALL_GRPC_FILES})
endif()

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
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

if(${BUILD_GRPC})
  set(ALL_GRPC_SERVICE_SRCS)
  set(ALL_GRPC_SERVICE_HDRS)
  foreach(ABS_FIL ${ALL_GRPC_SERVICES})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out  ${PROJECT_BINARY_DIR}
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

# Generate source file with version definitions.
include("${PROJECT_SOURCE_DIR}/cmake/generate_version_definitions.cmake")

## Define Eigen addons. 
include(${PROJECT_SOURCE_DIR}/cmake/grid_map_core-extras.cmake)

# Thirdparty library
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)

# Thirdparty: class_loader
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/autonomy_class_loader_vendor/class_loader)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/autonomy_class_loader_vendor/class_loader/include)

# # gRPC
# if(${BUILD_GRPC})
#   add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/autonomy_async_grpc_vendor/async_grpc)
#   include_directories(${PROJECT_SOURCE_DIR}/3rdparty/autonomy_async_grpc_vendor/async_grpc/include)
# endif()

# # include visualization
# if(${BUILD_FOXGLOVE})
#   if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
#     set(FOXGLOVE_SDK_RELEASES "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.10.0/foxglove-v0.10.0-cpp-aarch64-unknown-linux-gnu.zip" "81e2379e3a08160c023779eab0fe5c5764ace3b30c8727b0030ac7dc8b415016")
#   elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
#     set(FOXGLOVE_SDK_RELEASES "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.10.0/foxglove-v0.10.0-cpp-x86_64-unknown-linux-gnu.zip" "209d34a8703d44a129c559493e1a3fb215888b4d5973622750ac28b1c345946c")
#   else()
#     message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")
#   endif()

#   list(GET FOXGLOVE_SDK_RELEASES 0 url)
#   list(GET FOXGLOVE_SDK_RELEASES 1 hash)

#   # Download Foxglove SDK shared lib
#   include(FetchContent)
#   FetchContent_Declare(
#     foxglove_sdk
#     URL ${url}
#     URL_HASH SHA256=${hash}
#   )
#   FetchContent_MakeAvailable(foxglove_sdk)

#   add_library(foxglove_sdk STATIC)
#   target_include_directories(foxglove_sdk SYSTEM PUBLIC
#       $<BUILD_INTERFACE:${foxglove_sdk_SOURCE_DIR}/include>
#       $<INSTALL_INTERFACE:include>
#   )
#   file(GLOB_RECURSE FOXGLOVE_SDK_SOURCES CONFIGURE_DEPENDS "${foxglove_sdk_SOURCE_DIR}/src/*.cpp")
#   target_sources(foxglove_sdk PUBLIC ${FOXGLOVE_SDK_SOURCES})
#   set_target_properties(foxglove_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
#   target_link_libraries(foxglove_sdk PUBLIC ${foxglove_sdk_SOURCE_DIR}/lib/libfoxglove.a)
# endif()

# Autonomy library
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

# protobuf
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY})

# Eigen3
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

# Ceres solver
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

# DDS(CyberRT & FastRTPS)
if (${BUILD_CYBER})
  # CyberRT
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CYBERRT_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${CYBERRT_LIBRARY})

  # FastRTPS
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${FastRTPS_INCLUDE_DIR}")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${FastRTPS_LIBRARY})
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

# glog & tf2 & eventpp & nlohmann_json
set(GLOG_LIBRARY glog)
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

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
  # google_binary(cartographer_grpc_server
  #   SRCS
  #     cartographer/cloud/map_builder_server_main.cc
  # )
  target_link_libraries(${PROJECT_NAME} PUBLIC grpc++)
  target_link_libraries(${PROJECT_NAME} PUBLIC async_grpc)
  # if(${BUILD_PROMETHEUS})
  #   target_link_libraries(cartographer_grpc_server PUBLIC ${ZLIB_LIBRARIES})
  #   target_link_libraries(cartographer_grpc_server PUBLIC prometheus-cpp-core)
  #   target_link_libraries(cartographer_grpc_server PUBLIC prometheus-cpp-pull)
  # endif()
endif()

# if(${BUILD_FOXGLOVE})
#   target_include_directories(${PROJECT_NAME} PRIVATE ${foxglove_sdk_SOURCE_DIR}/include)
#   target_link_libraries(${PROJECT_NAME} PRIVATE foxglove_sdk)
# endif()

# autonomy.system.launcher
google_binary(autonomy.system.launcher
  SRCS
    autonomy/system/autonomy_main.cpp
)

# autonomy.tasks.navigator
google_binary(autonomy.tasks.navigator
  SRCS
    autonomy/tasks/navigator_main.cpp
)

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
  if(${BUILD_GRPC})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++)
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC async_grpc)
  endif()
  if(${BUILD_PROMETHEUS})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${ZLIB_LIBRARIES})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-core)
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-pull)
  endif()
  target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
endforeach()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT AutonomyExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

install(
  EXPORT AutonomyExport
  DESTINATION share/autonomy/cmake/
  FILE AutonomyTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/autonomy-config.cmake
  DESTINATION share/autonomy/
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/autonomy/launch 
  DESTINATION launch)