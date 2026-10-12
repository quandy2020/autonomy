# Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
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
#
# Standalone: cmake -S automanip/bolza · Optional nest: add_subdirectory from automanip/
# No link/include dependency on autonomy_* / autocmake / autolink.

cmake_minimum_required(VERSION 3.20)
set(BOLZA_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(bolza LANGUAGES CXX)
endif()

option(BOLZA_WITH_ONNX "mpcnet ONNX when SDK found" ON)
option(BOLZA_BUILD_TESTS "GTest unit tests" OFF)

include(cmake/functions.cmake)
include(cmake/bolza_version.cmake)
bolza_collect_git_version()
bolza_initialize_project()

configure_file(
  config/conf.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/bolza/conf/conf.hpp
  @ONLY)

# --- deps --------------------------------------------------------------------
find_package(OpenMP QUIET)
find_package(Eigen3 QUIET)
find_package(Threads QUIET)
find_package(CppAD QUIET)
find_package(Pinocchio QUIET)
find_package(GLPK QUIET)
if(NOT (OpenMP_FOUND AND Eigen3_FOUND AND Threads_FOUND AND CppAD_FOUND
        AND Pinocchio_FOUND AND GLPK_FOUND))
  message(STATUS "bolza: skipped (need Eigen, Threads, OpenMP, CppAD, Pinocchio, GLPK)")
  return()
endif()
include(cmake/bolza_hpipm.cmake)

if(BOLZA_WITH_ONNX)
  find_package(ONNXRuntime QUIET)
endif()
set(BOLZA_HAS_ONNX ${ONNXRuntime_FOUND})
find_package(Coal QUIET)
set(BOLZA_HAS_COAL ${Coal_FOUND})

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE ALL_LIBRARY_SRCS CONFIGURE_DEPENDS "bolza/*.cpp")
file(GLOB_RECURSE ALL_TESTS CONFIGURE_DEPENDS "bolza/*_test.cpp")
file(GLOB_RECURSE TEST_LIBRARY_SRCS CONFIGURE_DEPENDS "bolza/ddp/bouncingmass/*.cpp")

list(FILTER ALL_LIBRARY_SRCS EXCLUDE REGEX "_test\\.cpp$|/ddp/bouncingmass/")
list(FILTER TEST_LIBRARY_SRCS EXCLUDE REGEX "_test\\.cpp$")
list(FILTER ALL_TESTS EXCLUDE REGEX "/constraints_demo_test\\.cpp$")
if(NOT BOLZA_HAS_ONNX)
  list(FILTER ALL_LIBRARY_SRCS EXCLUDE REGEX "/mpcnet_onnx_controller\\.cpp$")
endif()
if(NOT BOLZA_HAS_COAL)
  list(FILTER ALL_LIBRARY_SRCS EXCLUDE REGEX "/pinocchio/(self_collision|sphere)/")
  list(FILTER ALL_TESTS EXCLUDE REGEX "/pinocchio/(self_collision|sphere)/")
endif()

# --- library -----------------------------------------------------------------
add_library(bolza SHARED ${ALL_LIBRARY_SRCS})
add_library(bolza::bolza ALIAS bolza)

set_target_properties(bolza PROPERTIES
  VERSION "${BOLZA_VERSION}"
  SOVERSION "${BOLZA_SOVERSION}"
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)

target_include_directories(bolza PUBLIC $<BUILD_INTERFACE:${BOLZA_ROOT_DIR}>)
target_include_directories(bolza PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_include_directories(bolza PUBLIC $<INSTALL_INTERFACE:include>)

target_compile_definitions(bolza PUBLIC PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR)
target_compile_definitions(bolza PUBLIC PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

target_link_libraries(bolza PUBLIC Threads::Threads)
target_link_libraries(bolza PUBLIC Eigen3::Eigen)
target_link_libraries(bolza PUBLIC OpenMP::OpenMP_CXX)
target_link_libraries(bolza PUBLIC CppAD::CppAD)
target_link_libraries(bolza PUBLIC pinocchio::pinocchio)
target_link_libraries(bolza PUBLIC GLPK::GLPK)
target_link_libraries(bolza PUBLIC bolza_hpipm_deps)

target_compile_options(bolza PUBLIC -Wno-ignored-attributes)
target_compile_options(bolza PUBLIC -Wno-invalid-partial-specialization)

if(BOLZA_HAS_ONNX)
  target_compile_definitions(bolza PUBLIC BOLZA_HAS_ONNX=1)
  target_link_libraries(bolza PUBLIC onnxruntime::onnxruntime)
endif()
if(BOLZA_HAS_COAL)
  target_compile_definitions(bolza PUBLIC
    BOLZA_HAS_COAL=1
    COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
    COAL_DISABLE_HPP_FCL_WARNINGS)
  target_link_libraries(bolza PUBLIC coal::coal)
endif()

message(STATUS "bolza: ON (coal=${BOLZA_HAS_COAL} onnx=${BOLZA_HAS_ONNX})")

# --- tests -------------------------------------------------------------------
if(BOLZA_BUILD_TESTS)
  bolza_enable_testing()
  set(BOLZA_ROBOTIC_ASSETS_ROOT "${BOLZA_ROOT_DIR}/bolza/robotic_assets" CACHE PATH
    "URDF / mesh resources root")

  add_library(bolza_test_library ${TEST_LIBRARY_SRCS})
  target_link_libraries(bolza_test_library PUBLIC bolza)
  target_include_directories(bolza_test_library PUBLIC
    "${BOLZA_ROOT_DIR}/bolza"
    "${BOLZA_ROOT_DIR}/bolza/core"
    "${BOLZA_ROOT_DIR}/bolza/oc"
    "${BOLZA_ROOT_DIR}/bolza/ddp"
    "${BOLZA_ROOT_DIR}/bolza/qp"
    "${BOLZA_ROOT_DIR}/bolza/pinocchio/centroidal_model")

  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${BOLZA_ROOT_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    bolza_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC bolza_test_library)
    target_compile_definitions("${TEST_TARGET_NAME}" PRIVATE
      BOLZA_ROBOTIC_ASSETS_ROOT="${BOLZA_ROBOTIC_ASSETS_ROOT}")
  endforeach()
endif()

# --- install -----------------------------------------------------------------
install(FILES package.xml solver_license.txt version.json DESTINATION share/bolza)
install(DIRECTORY cmake DESTINATION share/bolza/)
install(DIRECTORY bolza/
  DESTINATION include/bolza
  FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
    PATTERN "*_test.*" EXCLUDE
    PATTERN "helpers" EXCLUDE
    PATTERN "robotic_assets" EXCLUDE
    PATTERN "bouncingmass" EXCLUDE
    PATTERN "CMakeLists.txt" EXCLUDE)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/bolza/conf/conf.hpp
  DESTINATION include/bolza/conf)
install(TARGETS bolza ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

set(BOLZA_CMAKE_DIR share/bolza/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  bolza-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/bolza/bolza-config.cmake
  PATH_VARS BOLZA_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/bolza)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/bolza/bolza-config.cmake
  DESTINATION share/bolza/)
