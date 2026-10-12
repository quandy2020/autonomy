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

# Automanip: MoveIt-aligned planning stack + bolza/ OCP sub-library.

cmake_minimum_required(VERSION 3.20)

project(automanip VERSION 0.1.0 LANGUAGES CXX)

find_package(autocmake CONFIG QUIET)
if(NOT autocmake_FOUND)
  set(AUTOCMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autocmake")
  include("${AUTOCMAKE_DIR}/Autocmake.cmake")
endif()

option(AUTOMANIP_BUILD_TEST "Build unit tests" ON)
option(AUTOMANIP_BUILD_PLANNING
  "Build MoveIt-aligned planning stack (needs autonomy_common + protobuf)" ON)

if(DEFINED AUTOMANIP_BUILD_TEST AND NOT AUTOMANIP_BUILD_TEST)
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

autocmake_project()
autocmake_build(QUIT)

set(AUTOMANIP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(AUTOMANIP_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/automanip")

list(PREPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules")
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

# Planning stack needs autonomy_common (macros / conf_loader / logging).
set(AUTOMANIP_WITH_PLANNING OFF)
if(AUTOMANIP_BUILD_PLANNING)
  if(TARGET autonomy_common)
    set(AUTOMANIP_WITH_PLANNING ON)
  elseif(TARGET autonomy::common)
    set(AUTOMANIP_WITH_PLANNING ON)
  else()
    find_package(autonomy QUIET COMPONENTS common)
    if(TARGET autonomy_common OR TARGET autonomy::common)
      set(AUTOMANIP_WITH_PLANNING ON)
    else()
      message(STATUS "automanip: planning stack skipped (autonomy_common not found)")
    endif()
  endif()
endif()

file(GLOB_RECURSE AUTOMANIP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/automanip/*.cpp")
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/automanip/main\\.cpp$")
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/manipulation_main\\.cpp$")
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/setup/setup_main\\.cpp$")
list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "_test\\.cpp$")

if(NOT AUTOMANIP_WITH_PLANNING)
  list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX
    "/automanip/(common|constraints|dispatch|model|motion|pipeline|planner|setup|utils)/")
  list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX
    "/automanip/(manipulation_server|manipulation_options|plugin_registry)\\.cpp$")
endif()

# Optional SDK-backed sources (stubs stay when SDK absent).
if(AUTOMANIP_WITH_PLANNING)
  foreach(_pkg IN ITEMS OrocosKDL FCL OMPL Ruckig urdfdom Pinocchio VHACD TRAC_IK Octomap)
    autocmake_find(${_pkg} PRIVATE QUIET)
  endforeach()
  autocmake_find(opentelemetry-cpp PRIVATE QUIET CONFIG)
  if(NOT OrocosKDL_FOUND)
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/model/urdf_kdl_chain\\.cpp$")
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/motion/kinematics/kdl_kinematics\\.cpp$")
  endif()
  if(NOT FCL_FOUND)
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/motion/collision/fcl_collision_detector\\.cpp$")
  endif()
  if(OMPL_FOUND)
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/planner/ompl/ompl_planner_stub\\.cpp$")
  else()
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX "/planner/ompl/ompl_planner\\.cpp$")
  endif()
endif()

# Domain protos → generated under ${CMAKE_CURRENT_BINARY_DIR}/automanip/proto/
set(AUTOMANIP_PROTO_SRCS "")
if(AUTOMANIP_WITH_PLANNING)
  find_package(Protobuf QUIET)
  if(Protobuf_FOUND AND Protobuf_PROTOC_EXECUTABLE)
    file(GLOB AUTOMANIP_PROTO_FILES CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/automanip/proto/*.proto")
    set(_proto_incs -I "${CMAKE_CURRENT_SOURCE_DIR}" -I "${CMAKE_CURRENT_SOURCE_DIR}/..")
    if(DEFINED AUTOMSGS_PROTO_INCLUDE_DIR)
      list(APPEND _proto_incs -I "${AUTOMSGS_PROTO_INCLUDE_DIR}")
    endif()
    set(_proto_deps "")
    if(TARGET automsgs_proto_copy)
      set(_proto_deps automsgs_proto_copy)
    endif()
    foreach(_abs IN LISTS AUTOMANIP_PROTO_FILES)
      file(RELATIVE_PATH _rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_abs}")
      get_filename_component(_dir "${_rel}" DIRECTORY)
      get_filename_component(_we "${_rel}" NAME_WE)
      set(_base "${CMAKE_CURRENT_BINARY_DIR}/${_dir}/${_we}")
      add_custom_command(
        OUTPUT "${_base}.pb.cc" "${_base}.pb.h"
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
          --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
          ${_proto_incs} ${_abs}
        DEPENDS ${_abs} ${_proto_deps}
        COMMENT "protoc ${_rel}"
        VERBATIM)
      list(APPEND AUTOMANIP_PROTO_SRCS "${_base}.pb.cc")
    endforeach()
  else()
    message(STATUS "automanip: planning protos skipped (protobuf/protoc not found)")
    set(AUTOMANIP_WITH_PLANNING OFF)
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX
      "/automanip/(common|constraints|dispatch|model|motion|pipeline|planner|setup|utils)/")
    list(FILTER AUTOMANIP_SOURCES EXCLUDE REGEX
      "/automanip/(manipulation_server|manipulation_options|plugin_registry)\\.cpp$")
  endif()
endif()

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

set(_AUTOMANIP_DEPS
  Threads::Threads
  Eigen3::Eigen
  fkYAML::fkYAML
  autolink
  automsgs)
if(AUTOMANIP_WITH_PLANNING)
  if(TARGET autonomy_common)
    list(APPEND _AUTOMANIP_DEPS autonomy_common)
  elseif(TARGET autonomy::common)
    list(APPEND _AUTOMANIP_DEPS autonomy::common)
  endif()
  if(TARGET autonomy_transform)
    list(APPEND _AUTOMANIP_DEPS autonomy_transform)
  endif()
  list(APPEND _AUTOMANIP_DEPS
    $<TARGET_NAME_IF_EXISTS:protobuf::libprotobuf>
    $<TARGET_NAME_IF_EXISTS:OrocosKDL::orocos-kdl>
    $<TARGET_NAME_IF_EXISTS:FCL::fcl>
    $<TARGET_NAME_IF_EXISTS:OMPL::OMPL>
    $<TARGET_NAME_IF_EXISTS:ruckig::ruckig>
    $<TARGET_NAME_IF_EXISTS:urdfdom::urdfdom_model>
    $<TARGET_NAME_IF_EXISTS:pinocchio::pinocchio>
    $<TARGET_NAME_IF_EXISTS:VHACD::VHACD>
    $<TARGET_NAME_IF_EXISTS:TRAC_IK::TRAC_IK>
    $<TARGET_NAME_IF_EXISTS:octomap::octomap>
    $<TARGET_NAME_IF_EXISTS:opentelemetry-cpp::opentelemetry_trace>)
endif()

autocmake_library(automanip SHARED ${_AUTOMANIP_NO_EXPORT}
  SOURCES ${AUTOMANIP_SOURCES} ${AUTOMANIP_PROTO_SRCS}
  DEPENDENCIES ${_AUTOMANIP_DEPS})
set_target_properties(automanip PROPERTIES SOVERSION "${AUTOMANIP_SOVERSION}")
target_include_directories(automanip PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../autonomy/common/yaml_cpp_shim>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>)

if(AUTOMANIP_WITH_PLANNING)
  target_compile_definitions(automanip PUBLIC
    $<$<TARGET_EXISTS:OrocosKDL::orocos-kdl>:AUTOMANIP_HAS_KDL>
    $<$<TARGET_EXISTS:FCL::fcl>:AUTOMANIP_HAS_FCL>
    $<$<TARGET_EXISTS:OMPL::OMPL>:AUTOMANIP_HAS_OMPL>
    $<$<TARGET_EXISTS:ruckig::ruckig>:AUTOMANIP_HAS_RUCKIG>
    $<$<TARGET_EXISTS:urdfdom::urdfdom_model>:AUTOMANIP_HAS_URDFDOM>
    $<$<TARGET_EXISTS:pinocchio::pinocchio>:AUTOMANIP_HAS_PINOCCHIO>
    $<$<TARGET_EXISTS:VHACD::VHACD>:AUTOMANIP_HAS_VHACD>
    $<$<TARGET_EXISTS:TRAC_IK::TRAC_IK>:AUTOMANIP_HAS_TRAC_IK>
    $<$<TARGET_EXISTS:octomap::octomap>:AUTOMANIP_HAS_OCTOMAP>
    $<$<TARGET_EXISTS:opentelemetry-cpp::opentelemetry_trace>:AUTOMANIP_HAVE_OTEL=1>)
  message(STATUS "automanip: building MoveIt-aligned planning stack")
endif()

if(AUTOMANIP_WITH_PLANNING)
  # Primary process: ManipulationServer (former arm HAL entry removed).
  autocmake_binary(automanip_main
    SOURCES automanip/main.cpp options.cpp
    DEPENDENCIES automanip glog::glog)
  set_target_properties(automanip_main PROPERTIES OUTPUT_NAME automanip)
  target_include_directories(automanip_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include
    ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET autonomy::cli11)
    target_link_libraries(automanip_main PRIVATE autonomy::cli11)
  elseif(TARGET autonomy)
    target_link_libraries(automanip_main PRIVATE autonomy)
  endif()

  autocmake_binary(automanip.manipulation
    SOURCES automanip/manipulation_main.cpp
    DEPENDENCIES automanip glog::glog)
  target_include_directories(automanip.manipulation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include)
  if(TARGET autonomy::cli11)
    target_link_libraries(automanip.manipulation PRIVATE autonomy::cli11)
  elseif(TARGET autonomy)
    target_link_libraries(automanip.manipulation PRIVATE autonomy)
  endif()

  autocmake_binary(automanip.manipulation.setup
    SOURCES automanip/setup/setup_main.cpp
    DEPENDENCIES automanip glog::glog)
  target_include_directories(automanip.manipulation.setup PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include)
  if(TARGET autonomy::cli11)
    target_link_libraries(automanip.manipulation.setup PRIVATE autonomy::cli11)
  elseif(TARGET autonomy)
    target_link_libraries(automanip.manipulation.setup PRIVATE autonomy)
  endif()
endif()

if(AUTOMANIP_BUILD_TEST)
  autocmake_test(test_automanip_config
    SOURCES test/test_config_loader.cpp
    DEPENDENCIES automanip)
  if(AUTOMANIP_WITH_PLANNING)
    foreach(_t IN ITEMS
        urdf_joints_test:automanip/model/urdf_joints_test.cpp
        plugins_test:automanip/plugins_test.cpp
        time_parameterization_test:automanip/pipeline/time_parameterization_test.cpp
        aabb_collision_detector_test:automanip/motion/collision/aabb_collision_detector_test.cpp
        link_collision_geometry_test:automanip/motion/collision/link_collision_geometry_test.cpp
        servo_test:automanip/motion/servo/cartesian_servo_test.cpp
        effort_tracking_controller_test:automanip/motion/execution/effort_tracking_controller_test.cpp)
      string(REPLACE ":" ";" _pair "${_t}")
      list(GET _pair 0 _name)
      list(GET _pair 1 _src)
      autocmake_test(automanip.${_name}
        SOURCES ${_src}
        DEPENDENCIES automanip)
    endforeach()
    if(OrocosKDL_FOUND)
      autocmake_test(automanip.kdl_kinematics_test
        SOURCES automanip/motion/kinematics/kdl_kinematics_test.cpp
        DEPENDENCIES automanip)
    endif()
    if(OMPL_FOUND)
      autocmake_test(automanip.ompl_planner_test
        SOURCES automanip/planner/ompl/ompl_planner_test.cpp
        DEPENDENCIES automanip)
    endif()
    if(FCL_FOUND)
      autocmake_test(automanip.fcl_collision_detector_test
        SOURCES automanip/motion/collision/fcl_collision_detector_test.cpp
        DEPENDENCIES automanip)
    endif()
  endif()
endif()

# Bolza OCP sub-library (libbolza + version/conf). Optional if deps missing.
# libbolza itself has no autolink dependency; demos live under examples/.
add_subdirectory(bolza)

add_subdirectory(automanip)
add_subdirectory(examples)
add_subdirectory(config)
add_subdirectory(dag)
add_subdirectory(launch)

# Planning-stack share (conf / dag / launch / test_data) under share/automanip/.
if(AUTOMANIP_WITH_PLANNING)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/automanip/conf/"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/conf"
    FILES_MATCHING PATTERN "*.pb.txt" PATTERN "*.plugins")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/automanip/dag/"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/dag"
    FILES_MATCHING PATTERN "*.dag")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/automanip/launch/"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/launch"
    FILES_MATCHING PATTERN "*.launch")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/automanip/test_data/"
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/test_data")
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  autocmake_package()
endif()
