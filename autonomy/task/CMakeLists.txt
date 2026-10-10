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
