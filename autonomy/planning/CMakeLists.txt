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

# autonomy/planning — global / local planners

include(autonomy_module)
autonomy_module(planning)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_PLANNING_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_map
  SRCS ${_PLANNING_SRCS}
)
unset(_PLANNING_SRCS)

# Process entrypoint: autonomy.planning
autonomy_binary(autonomy.planning SRCS planning_main.cpp)

# Optional planning tools (if present).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/CMakeLists.txt")
  add_subdirectory(tools)
endif()

# Runtime assets → share/autonomy/planning/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/planning/conf)
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/planning/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/planning/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
