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
  "${CMAKE_CURRENT_SOURCE_DIR}/automanip/*.cpp")
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/automanip/main\\.cpp$")
# Solver sources need CppAD and Boost.Log. They are added below when those
# headers are present, so a machine without them still builds the arm HAL.
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX
  "/automanip/(core|oc|mpc|ddp|qp|slp|tools|ipm|mpcnet|sqp|pinocchio|perceptive)/")

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

set(_AUTOMANIP_CPPAD_DEFAULT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../ocs2/ocs2_thirdparty/include")
set(AUTOMANIP_CPPAD_INCLUDE "${_AUTOMANIP_CPPAD_DEFAULT}"
  CACHE PATH "Directory containing cppad/cg.hpp")
find_package(Boost QUIET COMPONENTS log log_setup filesystem system)
find_package(OpenMP QUIET)
set(AUTOMANIP_WITH_SOLVER OFF)
if(Boost_FOUND AND OpenMP_FOUND AND EXISTS "${AUTOMANIP_CPPAD_INCLUDE}/cppad/cg.hpp")
  file(GLOB_RECURSE AUTOMANIP_SOLVER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/oc/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/mpc/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/ddp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/qp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/slp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/ipm/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/mpcnet/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/sqp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/pinocchio/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/tools/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/automanip/perceptive/*.cpp")
  # The ONNX policy controller needs ONNX Runtime.
  find_path(AUTOMANIP_ONNXRUNTIME_INCLUDE onnxruntime/onnxruntime_cxx_api.h)
  if(NOT AUTOMANIP_ONNXRUNTIME_INCLUDE)
    list(FILTER AUTOMANIP_SOLVER_SOURCES EXCLUDE REGEX "/mpcnet_onnx_controller\\.cpp$")
    message(STATUS "automanip: mpcnet ONNX controller skipped (onnxruntime headers not found)")
  endif()
  find_package(PkgConfig QUIET)
  set(AUTOMANIP_WITH_PINOCCHIO OFF)
  if(PkgConfig_FOUND)
    pkg_check_modules(pinocchio QUIET pinocchio)
    pkg_check_modules(hppfcl QUIET hpp-fcl)
  endif()
  if(NOT pinocchio_FOUND)
    list(FILTER AUTOMANIP_SOLVER_SOURCES EXCLUDE REGEX "/automanip/pinocchio/")
    message(STATUS "automanip: pinocchio sources skipped (pinocchio not found)")
  else()
    set(AUTOMANIP_WITH_PINOCCHIO ON)
    if(NOT hppfcl_FOUND)
      list(FILTER AUTOMANIP_SOLVER_SOURCES EXCLUDE REGEX "/pinocchio/(self_collision|sphere)/")
      message(STATUS "automanip: pinocchio collision and sphere sources skipped (hpp-fcl not found)")
    endif()
  endif()
  target_sources(automanip PRIVATE ${AUTOMANIP_SOLVER_SOURCES})
  target_compile_definitions(automanip PUBLIC BOOST_ALL_DYN_LINK)
  target_include_directories(automanip PUBLIC "${AUTOMANIP_CPPAD_INCLUDE}")
  if(AUTOMANIP_ONNXRUNTIME_INCLUDE)
    target_include_directories(automanip PUBLIC "${AUTOMANIP_ONNXRUNTIME_INCLUDE}")
    target_link_libraries(automanip PUBLIC onnxruntime)
  endif()
  if(AUTOMANIP_WITH_PINOCCHIO)
    target_include_directories(automanip PUBLIC ${pinocchio_INCLUDE_DIRS})
    target_link_directories(automanip PUBLIC ${pinocchio_LIBRARY_DIRS})
    target_link_libraries(automanip PUBLIC ${pinocchio_LIBRARIES})
    target_compile_definitions(automanip PUBLIC
      PINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
      PINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)
    target_compile_options(automanip PUBLIC
      ${pinocchio_CFLAGS_OTHER}
      -Wno-ignored-attributes
      -Wno-invalid-partial-specialization)
    if(hppfcl_FOUND)
      target_include_directories(automanip PUBLIC ${hppfcl_INCLUDE_DIRS})
      target_link_directories(automanip PUBLIC ${hppfcl_LIBRARY_DIRS})
      target_link_libraries(automanip PUBLIC ${hppfcl_LIBRARIES})
    endif()
  endif()
  target_link_libraries(automanip PUBLIC
    Boost::log
    Boost::log_setup
    Boost::filesystem
    Boost::system
    OpenMP::OpenMP_CXX)
  set(AUTOMANIP_WITH_SOLVER ON)
  message(STATUS "automanip: building solver sources from automanip/{core,oc,mpc,ddp,qp,slp,ipm,mpcnet,sqp,pinocchio,tools,perceptive}")
else()
  message(STATUS "automanip: solver sources skipped (need Boost log/filesystem, OpenMP, and cppad/cg.hpp)")
endif()

autocmake_binary(automanip_main
  SOURCES automanip/main.cpp options.cpp
  DEPENDENCIES automanip)
set_target_properties(automanip_main PROPERTIES OUTPUT_NAME automanip)
target_include_directories(automanip_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include
  ${CMAKE_CURRENT_BINARY_DIR})

if(AUTOMANIP_BUILD_TEST)
  autocmake_test(test_automanip_config
    SOURCES test/test_config_loader.cpp
    DEPENDENCIES automanip)
endif()

add_subdirectory(automanip)
add_subdirectory(examples)
add_subdirectory(config)
add_subdirectory(dag)
add_subdirectory(launch)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  autocmake_package()
endif()
