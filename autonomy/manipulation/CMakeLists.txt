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

# autonomy/manipulation — MoveIt2-aligned arm stack (planning-style layout)

include(autonomy_module)
autonomy_module(manipulation)

autonomy_glob_srcs(_MANIPULATION_SRCS)

set(_MANIPULATION_FEATURES "")
if(OrocosKDL_FOUND)
  list(APPEND _MANIPULATION_FEATURES kdl)
else()
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/model/urdf_kdl\\.cpp$")
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/motion/kinematics/kdl_kinematics\\.cpp$")
endif()

if(FCL_FOUND)
  list(APPEND _MANIPULATION_FEATURES fcl)
else()
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/motion/collision/fcl_collision_detector\\.cpp$")
endif()

if(OMPL_FOUND)
  list(APPEND _MANIPULATION_FEATURES ompl)
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/planner/ompl/ompl_planner_stub\\.cpp$")
else()
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/planner/ompl/ompl_planner\\.cpp$")
endif()

if(Ruckig_FOUND)
  list(APPEND _MANIPULATION_FEATURES ruckig)
endif()

if(urdfdom_FOUND)
  list(APPEND _MANIPULATION_FEATURES urdfdom)
endif()

if(Pinocchio_FOUND)
  list(APPEND _MANIPULATION_FEATURES pinocchio)
endif()

if(VHACD_FOUND)
  list(APPEND _MANIPULATION_FEATURES vhacd)
endif()

if(TRAC_IK_FOUND)
  list(APPEND _MANIPULATION_FEATURES trac_ik)
endif()

if(Octomap_FOUND)
  list(APPEND _MANIPULATION_FEATURES octomap)
endif()

autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_proto
  FEATURES ${_MANIPULATION_FEATURES}
  SRCS ${_MANIPULATION_SRCS}
)
if(_MANIPULATION_FEATURES)
  message(STATUS "autonomy_manipulation FEATURES: ${_MANIPULATION_FEATURES}")
else()
  message(STATUS "autonomy_manipulation FEATURES: (none — AABB/stub backends)")
endif()
unset(_MANIPULATION_SRCS)
unset(_MANIPULATION_FEATURES)

autonomy_binary(autonomy.manipulation SRCS manipulation_main.cpp)
autonomy_binary(autonomy.manipulation.setup SRCS setup/setup_main.cpp)

if(BUILD_TEST)
  autonomy_test(manipulation.joint_interpolation_planner_test
    planner/joint_interpolation/joint_interpolation_planner_test.cpp
    autonomy_manipulation)
  autonomy_test(manipulation.urdf_joints_test
    model/urdf_joints_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.plugins_test
    plugins_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.time_parameterization_test
    planner/pipeline/time_parameterization_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.aabb_collision_detector_test
    motion/collision/aabb_collision_detector_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.link_collision_geometry_test
    motion/collision/link_collision_geometry_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.servo_test
    motion/servo/servo_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.effort_tracking_controller_test
    motion/execution/effort_tracking_controller_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.plan_benchmark_test
    benchmarks/plan_benchmark_test.cpp autonomy_manipulation)
  if(OrocosKDL_FOUND)
    autonomy_test(manipulation.kdl_kinematics_test
      motion/kinematics/kdl_kinematics_test.cpp autonomy_manipulation)
  endif()
  if(OMPL_FOUND)
    autonomy_test(manipulation.ompl_planner_test
      planner/ompl/ompl_planner_test.cpp autonomy_manipulation)
  endif()
  if(FCL_FOUND)
    autonomy_test(manipulation.fcl_collision_detector_test
      motion/collision/fcl_collision_detector_test.cpp autonomy_manipulation)
  endif()
endif()

install(
  DIRECTORY conf/
  DESTINATION share/autonomy/manipulation/conf)
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/manipulation/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/manipulation/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
install(
  DIRECTORY test_data/
  DESTINATION share/autonomy/manipulation/test_data)
