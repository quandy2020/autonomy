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

# autonomy/visualization — Foxglove bridge (skipped if foxglove-sdk absent)

include(autonomy_module)
autonomy_module(visualization)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_VISUALIZATION_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_map
  FEATURES foxglove
  SRCS ${_VISUALIZATION_SRCS}
)
unset(_VISUALIZATION_SRCS)

# Process entrypoint: autonomy.foxglove_bridge (when foxglove_main.cpp exists).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/foxglove_main.cpp")
  autonomy_binary(autonomy.foxglove_bridge SRCS foxglove_main.cpp)
endif()
