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

# autonomy/localization — SLAM / pose estimation (Atlas, Cartographer, …)

include(autonomy_module)
autonomy_module(localization)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_LOCALIZATION_SRCS)
list(FILTER _LOCALIZATION_SRCS EXCLUDE REGEX "/(fake_|mock_)")
list(FILTER _LOCALIZATION_SRCS EXCLUDE REGEX "test_helpers")
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform
  FEATURES slam pcl cairo boost_iostreams lua
  SRCS ${_LOCALIZATION_SRCS}
)
unset(_LOCALIZATION_SRCS)

# Process entrypoint: autonomy.localization
autonomy_binary(autonomy.localization SRCS localization_main.cpp)

# Offline EuRoC mono-inertial runner (Atlas VIO)
autonomy_binary(autonomy.localization.euroc_vio
  SRCS atlas/example/euroc/euroc_inertial_main.cpp)

# Runtime assets → share/autonomy/localization/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/localization/conf
)
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/localization/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/localization/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
