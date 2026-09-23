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

# autonomy/localization — SLAM / pose estimation (atlas, Cartographer, lightning, …)

include(autonomy_module)
autonomy_module(localization)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_LOCALIZATION_SRCS)
list(FILTER _LOCALIZATION_SRCS EXCLUDE REGEX "/(fake_|mock_)")
list(FILTER _LOCALIZATION_SRCS EXCLUDE REGEX "test_helpers")
# Atlas: do not compile vendored / tool trees into the library.
list(FILTER _LOCALIZATION_SRCS EXCLUDE REGEX "/atlas/thirdparty/")

set(_LOCALIZATION_FEATURES slam pcl cairo boost_iostreams lua)

# lightning-lm original include roots (must exist before compiling sources).
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lightning
  ${CMAKE_CURRENT_SOURCE_DIR}/lightning/thirdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/lightning/core/miao
)

autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform
  FEATURES ${_LOCALIZATION_FEATURES}
  SRCS ${_LOCALIZATION_SRCS}
)
unset(_LOCALIZATION_SRCS)
unset(_LOCALIZATION_FEATURES)

# lightning-lm: original include layout (common/, core/, Sophus/, miao core/).
target_include_directories(autonomy_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lightning>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lightning/thirdparty>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lightning/core/miao>
)
target_compile_definitions(autonomy_localization PUBLIC PCL_NO_PRECOMPILE)

# Process entrypoint: autonomy.localization
autonomy_binary(autonomy.localization SRCS localization_main.cpp)

# Atlas: cli / node binaries + config install
add_subdirectory(atlas)

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
