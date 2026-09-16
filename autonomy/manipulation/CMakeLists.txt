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

# autonomy/manipulation — MoveIt2-aligned arm stack (A+②)

include(autonomy_module)
autonomy_module(manipulation)

autonomy_glob_srcs(_MANIPULATION_SRCS)

set(_MANIPULATION_FEATURES "")
if(OrocosKDL_FOUND)
  list(APPEND _MANIPULATION_FEATURES kdl)
else()
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/core/urdf_kdl\\.cpp$")
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/kinematics/kdl_kinematics\\.cpp$")
endif()

if(FCL_FOUND)
  list(APPEND _MANIPULATION_FEATURES fcl)
else()
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/collision/fcl_collision_detector\\.cpp$")
endif()

if(OMPL_FOUND)
  list(APPEND _MANIPULATION_FEATURES ompl)
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/planning/ompl_planner_stub\\.cpp$")
else()
  list(FILTER _MANIPULATION_SRCS EXCLUDE REGEX "/planning/ompl_planner\\.cpp$")
endif()

if(Ruckig_FOUND)
  list(APPEND _MANIPULATION_FEATURES ruckig)
endif()

if(urdfdom_FOUND)
  list(APPEND _MANIPULATION_FEATURES urdfdom)
endif()

autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_proto
  FEATURES ${_MANIPULATION_FEATURES}
  SRCS ${_MANIPULATION_SRCS}
)
unset(_MANIPULATION_SRCS)
unset(_MANIPULATION_FEATURES)

autonomy_binary(autonomy.manipulation SRCS manipulation_main.cpp)

if(BUILD_TEST)
  autonomy_test(manipulation.joint_interpolation_planner_test
    planning/joint_interpolation_planner_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.urdf_joints_test
    core/urdf_joints_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.plugins_test
    plugins_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.time_parameterization_test
    planning/time_parameterization_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.aabb_collision_detector_test
    collision/aabb_collision_detector_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.servo_test
    servo/servo_test.cpp autonomy_manipulation)
  autonomy_test(manipulation.plan_benchmark_test
    benchmarks/plan_benchmark_test.cpp autonomy_manipulation)
  if(OrocosKDL_FOUND)
    autonomy_test(manipulation.kdl_kinematics_test
      kinematics/kdl_kinematics_test.cpp autonomy_manipulation)
  endif()
  if(OMPL_FOUND)
    autonomy_test(manipulation.ompl_planner_test
      planning/ompl_planner_test.cpp autonomy_manipulation)
  endif()
  if(FCL_FOUND)
    autonomy_test(manipulation.fcl_collision_detector_test
      collision/fcl_collision_detector_test.cpp autonomy_manipulation)
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
