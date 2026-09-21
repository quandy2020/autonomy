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

# autonomy/system — shared AutonomyOptions loader + system monitor

include(autonomy_module)
autonomy_module(system)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_SYSTEM_SRCS)
autonomy_library(
  DEPENDENCIES
    autonomy_common
    autonomy_task
  FEATURES prometheus
  SRCS ${_SYSTEM_SRCS}
)
unset(_SYSTEM_SRCS)

# Process entrypoint: autonomy.monitor (multi-process stack via launch).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/monitor/monitor_main.cpp")
  autonomy_binary(autonomy.monitor SRCS monitor/monitor_main.cpp)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ota/ota_main.cpp")
  autonomy_binary(autonomy.ota SRCS ota/ota_main.cpp)
endif()

# Runtime assets → share/autonomy/system/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/system/conf)
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/system/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/system/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    DIRECTORY scripts/
    DESTINATION share/autonomy/system/scripts
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING PATTERN "*.sh")
endif()
