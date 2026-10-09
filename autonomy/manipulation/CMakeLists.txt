# Copyright 2026 The Openbot Authors
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

# -----------------------------------------------------------------------------
# autonomy/manipulation — MoveIt2-aligned arm stack
#
# Builds
#   autonomy_manipulation / autonomy::manipulation
#   autonomy.manipulation
#   autonomy.manipulation.setup
#
# Toggle
#   AUTONOMY_BUILD_MANIPULATION
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
foreach(_pkg IN ITEMS OrocosKDL FCL OMPL Ruckig urdfdom Pinocchio VHACD TRAC_IK Octomap)
  autocmake_find(${_pkg} PRIVATE QUIET)
endforeach()
autocmake_find(opentelemetry-cpp PRIVATE QUIET CONFIG)

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_MANIPULATION_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "_test\\.cpp$")

# Drop SDK-backed sources when the SDK is absent (stubs stay).
if(NOT OrocosKDL_FOUND)
  list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "/model/urdf_kdl_chain\\.cpp$")
  list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "/motion/kinematics/kdl_kinematics\\.cpp$")
endif()
if(NOT FCL_FOUND)
  list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "/motion/collision/fcl_collision_detector\\.cpp$")
endif()
if(OMPL_FOUND)
  list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "/planner/ompl/ompl_planner_stub\\.cpp$")
else()
  list(FILTER AUTONOMY_MANIPULATION_SRCS EXCLUDE REGEX "/planner/ompl/ompl_planner\\.cpp$")
endif()

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_manipulation
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_MANIPULATION_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    $<TARGET_NAME_IF_EXISTS:OrocosKDL::orocos-kdl>
    $<TARGET_NAME_IF_EXISTS:FCL::fcl>
    $<TARGET_NAME_IF_EXISTS:OMPL::OMPL>
    $<TARGET_NAME_IF_EXISTS:ruckig::ruckig>
    $<TARGET_NAME_IF_EXISTS:urdfdom::urdfdom_model>
    $<TARGET_NAME_IF_EXISTS:pinocchio::pinocchio>
    $<TARGET_NAME_IF_EXISTS:VHACD::VHACD>
    $<TARGET_NAME_IF_EXISTS:TRAC_IK::TRAC_IK>
    $<TARGET_NAME_IF_EXISTS:octomap::octomap>
    $<TARGET_NAME_IF_EXISTS:OSQP::OSQP>
    $<TARGET_NAME_IF_EXISTS:opentelemetry-cpp::opentelemetry_trace>)
add_library(autonomy::manipulation ALIAS autonomy_manipulation)

target_compile_definitions(autonomy_manipulation PUBLIC
  $<$<TARGET_EXISTS:OrocosKDL::orocos-kdl>:AUTONOMY_HAS_KDL>
  $<$<TARGET_EXISTS:FCL::fcl>:AUTONOMY_HAS_FCL>
  $<$<TARGET_EXISTS:OMPL::OMPL>:AUTONOMY_HAS_OMPL>
  $<$<TARGET_EXISTS:ruckig::ruckig>:AUTONOMY_HAS_RUCKIG>
  $<$<TARGET_EXISTS:urdfdom::urdfdom_model>:AUTONOMY_HAS_URDFDOM>
  $<$<TARGET_EXISTS:pinocchio::pinocchio>:AUTONOMY_HAS_PINOCCHIO>
  $<$<TARGET_EXISTS:VHACD::VHACD>:AUTONOMY_HAS_VHACD>
  $<$<TARGET_EXISTS:TRAC_IK::TRAC_IK>:AUTONOMY_HAS_TRAC_IK>
  $<$<TARGET_EXISTS:octomap::octomap>:AUTONOMY_HAS_OCTOMAP>
  $<$<TARGET_EXISTS:opentelemetry-cpp::opentelemetry_trace>:AUTONOMY_HAVE_OTEL=1>)

if(NOT TARGET OSQP::OSQP AND OSQP_LIBRARIES)
  target_link_libraries(autonomy_manipulation PUBLIC ${OSQP_LIBRARIES})
endif()

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.manipulation
  NO_EXPORT
  SOURCES manipulation_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    gflags::gflags)
autocmake_binary(autonomy.manipulation.setup
  NO_EXPORT
  SOURCES setup/setup_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    gflags::gflags)

if(BUILD_TEST OR BUILD_TESTING)
  foreach(_t IN ITEMS
      urdf_joints_test:model/urdf_joints_test.cpp
      plugins_test:plugins_test.cpp
      time_parameterization_test:pipeline/time_parameterization_test.cpp
      aabb_collision_detector_test:motion/collision/aabb_collision_detector_test.cpp
      link_collision_geometry_test:motion/collision/link_collision_geometry_test.cpp
      servo_test:motion/servo/cartesian_servo_test.cpp
      effort_tracking_controller_test:motion/execution/effort_tracking_controller_test.cpp)
    string(REPLACE ":" ";" _pair "${_t}")
    list(GET _pair 0 _name)
    list(GET _pair 1 _src)
    autocmake_test(manipulation.${_name}
      SOURCES ${_src}
      DEPENDENCIES autonomy_manipulation)
  endforeach()
  if(OrocosKDL_FOUND)
    autocmake_test(manipulation.kdl_kinematics_test
      SOURCES motion/kinematics/kdl_kinematics_test.cpp
      DEPENDENCIES autonomy_manipulation)
  endif()
  if(OMPL_FOUND)
    autocmake_test(manipulation.ompl_planner_test
      SOURCES planner/ompl/ompl_planner_test.cpp
      DEPENDENCIES autonomy_manipulation)
  endif()
  if(FCL_FOUND)
    autocmake_test(manipulation.fcl_collision_detector_test
      SOURCES motion/collision/fcl_collision_detector_test.cpp
      DEPENDENCIES autonomy_manipulation)
  endif()
endif()

# --- install -----------------------------------------------------------------
autocmake_install_share(manipulation
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
install(DIRECTORY test_data/
  DESTINATION share/autonomy/manipulation/test_data)
