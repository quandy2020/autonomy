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

# autonomy/vehicle — vehicle model / kinematics

include(autonomy_module)
autonomy_module(vehicle)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_VEHICLE_SRCS)
autonomy_library(
  NO_CORE
  DEPENDENCIES autonomy_common autonomy_proto
  SRCS ${_VEHICLE_SRCS}
)
unset(_VEHICLE_SRCS)
