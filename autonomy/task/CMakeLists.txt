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
# autonomy/task — task / BehaviorTree navigation
#
# Builds
#   autonomy_task / autonomy::task
#   autonomy.task
#
# Toggle
#   AUTONOMY_BUILD_TASK
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
autocmake_find(behaviortree_cpp REQUIRED PRIVATE
  EXTRA_ARGUMENTS PATHS /usr/local NO_DEFAULT_PATH)
if(NOT behaviortree_cpp_FOUND)
  message(FATAL_ERROR "task: behaviortree_cpp is required")
endif()

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_TASK_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_TASK_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_TASK_SRCS EXCLUDE REGEX "_test\\.cpp$")

option(AUTONOMY_BUILD_COVERAGE "Build native complete coverage task navigation" OFF)
if(AUTONOMY_BUILD_COVERAGE AND NOT AUTONOMY_BUILD_FIELDS2COVER)
  message(FATAL_ERROR "Coverage task requires AUTONOMY_BUILD_FIELDS2COVER=ON")
endif()
if(NOT AUTONOMY_BUILD_COVERAGE)
  list(FILTER AUTONOMY_TASK_SRCS EXCLUDE REGEX "/navigation/coverage/")
endif()

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_task
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_TASK_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_map
    autonomy_planning
    autonomy_control
    $<TARGET_NAME_IF_EXISTS:BT::behaviortree_cpp>
    $<TARGET_NAME_IF_EXISTS:behaviortree_cpp::behaviortree_cpp>)
add_library(autonomy::task ALIAS autonomy_task)

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.task
  NO_EXPORT
  SOURCES task_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

# --- install -----------------------------------------------------------------
autocmake_install_share(task
  CONF
  LAUNCH_PATTERN "*.launch")

autocmake_test(charging.auto_recharge_test
  SOURCES charging/auto_recharge_test.cpp
  DEPENDENCIES autonomy_task)

if(AUTONOMY_BUILD_COVERAGE)
  target_compile_definitions(autonomy_task PUBLIC AUTONOMY_BUILD_COVERAGE=1)
  target_compile_definitions(autonomy_task PRIVATE
    AUTONOMY_COVERAGE_DEFAULT_TREE="${CMAKE_CURRENT_SOURCE_DIR}/navigation/coverage/conf/navigate_complete_coverage.xml")
  install(DIRECTORY navigation/coverage/conf/ DESTINATION share/autonomy/task/navigation/coverage)
  add_executable(task.coverage_client navigation/coverage/examples/coverage_client.cc)
  target_link_libraries(task.coverage_client PRIVATE autonomy_task)
  option(AUTONOMY_BUILD_COVERAGE_TESTS "Build native coverage task regressions" ON)
  if(AUTONOMY_BUILD_COVERAGE_TESTS)
    enable_testing()
    add_executable(task.coverage_test navigation/coverage/tests/coverage_test.cpp)
    target_link_libraries(task.coverage_test PRIVATE autonomy_task)
    add_test(NAME task.coverage_test COMMAND task.coverage_test)
    set_tests_properties(task.coverage_test PROPERTIES TIMEOUT 120)

    add_executable(task.coverage_stress_test
        navigation/coverage/tests/coverage_stress_test.cpp)
    target_link_libraries(task.coverage_stress_test PRIVATE autonomy_task)
    add_test(NAME task.coverage_stress_test COMMAND task.coverage_stress_test)
    set_tests_properties(task.coverage_stress_test PROPERTIES TIMEOUT 900)
    add_executable(task.coverage_endpoint_test navigation/coverage/tests/coverage_endpoint_test.cpp)
    target_link_libraries(task.coverage_endpoint_test PRIVATE autonomy_task)
    add_test(NAME task.coverage_endpoint_test COMMAND task.coverage_endpoint_test)
    set_tests_properties(task.coverage_endpoint_test PROPERTIES TIMEOUT 30)

  endif()
endif()
