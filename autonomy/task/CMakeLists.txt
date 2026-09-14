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

# autonomy/task — task / BT navigation

include(autonomy_module)
autonomy_module(task)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_TASK_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_map autonomy_control
  FEATURES bt
  SRCS ${_TASK_SRCS}
)
unset(_TASK_SRCS)

# Process entrypoint: autonomy.task
autonomy_binary(autonomy.task SRCS task_main.cpp)

# Runtime assets → share/autonomy/task/
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/task/conf
)
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/task/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
