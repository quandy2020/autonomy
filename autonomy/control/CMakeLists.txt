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

# autonomy/control — motion controllers (TEB / MPPI / …); optional Ipopt

include(autonomy_module)
autonomy_module(control)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_CONTROL_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_map
  FEATURES ipopt
  SRCS ${_CONTROL_SRCS}
)
unset(_CONTROL_SRCS)

# Process entrypoint: autonomy.control
autonomy_binary(autonomy.control SRCS control_main.cpp)

# Runtime assets → share/autonomy/control/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/control/conf)
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/control/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/control/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
