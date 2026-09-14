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

# autonomy/sensor — sensor drivers / adapters

include(autonomy_module)
autonomy_module(sensor)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_SENSOR_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_control
  SRCS ${_SENSOR_SRCS}
)
unset(_SENSOR_SRCS)
