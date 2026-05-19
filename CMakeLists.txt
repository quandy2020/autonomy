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
find_package(Protobuf 3.0.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Adolc REQUIRED)
find_package(Ipopt QUIET)

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

set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/config
  CACHE PATH ".lua configuration files directory")
set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/autonomy/config
  CACHE PATH ".lua configuration files directory for cartographer")

install(DIRECTORY config DESTINATION share/autonomy)
install(DIRECTORY cmake DESTINATION share/autonomy/)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "autonomy/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "autonomy/*.cpp")
file(GLOB_RECURSE ALL_GRPC_HDRS "autonomy/common/async_grpc/*.hpp" "autonomy/bridge/plugins/grpc/*.hpp")
file(GLOB_RECURSE ALL_GRPC_SRCS "autonomy/common/async_grpc/*.cpp" "autonomy/bridge/plugins/grpc/*.cpp")
file(GLOB_RECURSE ALL_GRPC_FILES "autonomy/common/async_grpc/*" "autonomy/bridge/plugins/grpc/*")
file(GLOB_RECURSE ALL_GRPC_BRIDGE_FILES "autonomy/bridge/bridge_server.*")
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

set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "autonomy/*/internal/*.h" "autonomy/*/internal/*.hpp")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})

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

# Lua5.3
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

# OpenCV
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

# Yaml-cpp
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

# OSQP
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OSQP_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OSQP_LIBRARIES})

# glog
target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

# BehaviorTree.CPP (tasks module only; 4.7.x exports BT::behaviortree_cpp)
if(TARGET BT::behaviortree_cpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC BT::behaviortree_cpp)
elseif(TARGET behaviortree_cpp::behaviortree_cpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC behaviortree_cpp::behaviortree_cpp)
elseif(BUILD_TASKS)
  message(FATAL_ERROR
    "BUILD_TASKS is ON but behaviortree_cpp was not found. "
    "Install BehaviorTree.CPP or run install_behaviortree_cpp.sh.")
endif()

# prometheus (optional, for system/monitor and cartographer metrics/visualization)
if(${BUILD_PROMETHEUS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PROMETHEUS=1)
  find_package(prometheus-cpp CONFIG QUIET)
  if(prometheus-cpp_FOUND OR TARGET prometheus-cpp-core)
    target_link_libraries(${PROJECT_NAME} PRIVATE prometheus-cpp-core prometheus-cpp-pull)
  endif()
endif()

if(${BUILD_GRPC})
  target_link_libraries(${PROJECT_NAME} PUBLIC grpc++ grpc)
endif()

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