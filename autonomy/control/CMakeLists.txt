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

# -----------------------------------------------------------------------------
# autonomy/control — motion controllers (TEB / MPPI / …)
#
# Builds
#   autonomy_control / autonomy::control
#   autonomy.control
#
# Toggle
#   AUTONOMY_BUILD_CONTROL
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
autocmake_find(Ipopt PRIVATE QUIET)

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_CONTROL_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_CONTROL_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_CONTROL_SRCS EXCLUDE REGEX "_test\\.cpp$")

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_control
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_CONTROL_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_map
    $<TARGET_NAME_IF_EXISTS:Ipopt::Ipopt>)
add_library(autonomy::control ALIAS autonomy_control)

# Classic Ipopt package vars when no imported target exists.
if(Ipopt_FOUND AND NOT TARGET Ipopt::Ipopt)
  target_link_libraries(autonomy_control PUBLIC ${IPOPT_LIBRARIES})
endif()

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.control
  NO_EXPORT
  SOURCES control_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

# --- tests -------------------------------------------------------------------
# Root CMake keeps BUILD_TESTING cached OFF even when BUILD_TEST=ON.
if(BUILD_TEST AND NOT BUILD_TESTING)
  set(BUILD_TESTING ON)
endif()
autocmake_test(control.policy_action_schema_test
  SOURCES controller/policy_controller/action_schema_test.cpp
  DEPENDENCIES autonomy_control)
autocmake_test(control.policy_safety_filter_test
  SOURCES controller/policy_controller/safety_filter_test.cpp
  DEPENDENCIES autonomy_control)
autocmake_test(control.policy_path_encoder_test
  SOURCES controller/policy_controller/path_encoder_test.cpp
  DEPENDENCIES autonomy_control)
autocmake_test(control.policy_frame_align_test
  SOURCES controller/policy_controller/frame_align_test.cpp
  DEPENDENCIES autonomy_control)
autocmake_test(control.policy_sand_planner_test
  SOURCES controller/policy_controller/sand_planner_test.cpp
  DEPENDENCIES autonomy_control)
foreach(suite IN ITEMS robot_model nrmp planner obstacle_extractor)
  add_executable(control.neupan_${suite}_test
    controller/neupan_controller/${suite}_test.cpp)
  target_link_libraries(control.neupan_${suite}_test PRIVATE
    autonomy_control autonomy_common)
  add_test(NAME control.neupan_${suite}_test COMMAND control.neupan_${suite}_test)
  set_tests_properties(control.neupan_${suite}_test PROPERTIES TIMEOUT 60)
endforeach()

# These bounded scenario programs have their own main, rather than GTest cases.
# Register them directly so autocmake's GTest XML verifier is not applied and
# the native exploration gate can run while unrelated BUILD_TESTING stays OFF.
if(BUILD_TEST OR AUTONOMY_BUILD_EXPLORATION_TESTS)
  foreach(suite IN ITEMS differential_drive timed_trajectory local_ground local_trajectory_safety trajectory_tracker native_spline_tracker)
    add_executable(control.policy_${suite}_test
      controller/policy_controller/sand_planner/${suite}_test.cpp)
    target_link_libraries(control.policy_${suite}_test PRIVATE
      autonomy_control autonomy_common)
    add_test(NAME control.policy_${suite}_test COMMAND control.policy_${suite}_test)
    set_tests_properties(control.policy_${suite}_test PROPERTIES TIMEOUT 30)
  endforeach()
endif()

# --- install -----------------------------------------------------------------
autocmake_install_share(control
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
