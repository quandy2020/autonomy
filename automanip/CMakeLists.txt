# Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

# Manipulator HAL. This file builds libautomanip and the automanip process.
# The osc2 backend is the kinematic MPC ported from OCS2's fixed-base arm
# model (state = joint position, input = joint velocity).

cmake_minimum_required(VERSION 3.20)

project(automanip VERSION 0.1.0 LANGUAGES CXX)

find_package(autocmake CONFIG QUIET)
if(NOT autocmake_FOUND)
  set(AUTOCMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autocmake")
  include("${AUTOCMAKE_DIR}/Autocmake.cmake")
endif()

option(AUTOMANIP_BUILD_TEST "Build unit tests" ON)

if(DEFINED AUTOMANIP_BUILD_TEST AND NOT AUTOMANIP_BUILD_TEST)
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

autocmake_project()
autocmake_build(QUIT)

set(AUTOMANIP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(AUTOMANIP_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/automanip")

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(${AUTOMANIP_ROOT_DIR}/cmake/automanip_version.cmake)
automanip_collect_git_version()

set(AUTOMANIP_DEFAULT_CONFIG_DIR "${AUTOMANIP_ROOT_DIR}/config")
file(MAKE_DIRECTORY "${AUTOMANIP_GENERATED_DIR}/conf")
configure_file(
  "${AUTOMANIP_ROOT_DIR}/config/conf.hpp.in"
  "${AUTOMANIP_GENERATED_DIR}/conf/conf.hpp"
  @ONLY)

if(NOT TARGET autolink)
  find_package(autolink REQUIRED)
endif()
if(NOT TARGET automsgs)
  find_package(automsgs REQUIRED)
endif()

file(GLOB_RECURSE AUTOMANIP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/automanip/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/arm/*.cpp")
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/automanip/main\\.cpp$")

if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(_AUTOMANIP_NO_EXPORT NO_EXPORT)
endif()

if(NOT TARGET fkYAML::fkYAML)
  set(FK_YAML_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/fkYAML"
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/fkYAML"
    EXCLUDE_FROM_ALL)
endif()

autocmake_library(automanip SHARED ${_AUTOMANIP_NO_EXPORT}
  SOURCES ${AUTOMANIP_SOURCES}
  DEPENDENCIES
    Threads::Threads
    Eigen3::Eigen
    fkYAML::fkYAML
    autolink
    automsgs)
set_target_properties(automanip PROPERTIES SOVERSION "${AUTOMANIP_SOVERSION}")
target_include_directories(automanip PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../autonomy/common/yaml_cpp_shim>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>)

autocmake_binary(automanip_main
  SOURCES automanip/main.cpp options.cpp
  DEPENDENCIES automanip)
set_target_properties(automanip_main PROPERTIES OUTPUT_NAME automanip)
target_include_directories(automanip_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include
  ${CMAKE_CURRENT_BINARY_DIR})

if(AUTOMANIP_BUILD_TEST)
  autocmake_test(test_osc2
    SOURCES test/test_osc2.cpp
    DEPENDENCIES automanip)
  autocmake_test(test_arm_driver
    SOURCES test/test_arm_driver.cpp
    DEPENDENCIES automanip)
  autocmake_test(test_automanip_config
    SOURCES test/test_config_loader.cpp
    DEPENDENCIES automanip)
endif()

add_subdirectory(automanip)
add_subdirectory(arm)
add_subdirectory(config)
add_subdirectory(dag)
add_subdirectory(launch)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  autocmake_package()
endif()
