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

# autonomy/perception — perception stack

include(autonomy_module)
autonomy_module(perception)

# Library sources: GLOB *.cpp; exclude base/ and follow/ (built as components).
autonomy_glob_srcs(_PERCEPTION_SRCS)
list(FILTER _PERCEPTION_SRCS EXCLUDE REGEX "/(base|follow)/")

autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform autonomy_map
  FEATURES inference
  SRCS ${_PERCEPTION_SRCS}
)
unset(_PERCEPTION_SRCS)

# Process entrypoint: autonomy.perception
autonomy_binary(autonomy.perception SRCS perception_main.cpp)

# Loadable components (require an inference backend for base/).
set(_has_infer FALSE)
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  set(_has_infer TRUE)
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  set(_has_infer TRUE)
endif()
if(_has_infer AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/base/CMakeLists.txt")
  add_subdirectory(base)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/follow/CMakeLists.txt")
  add_subdirectory(follow)
endif()
unset(_has_infer)

# Runtime assets → share/autonomy/perception/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/perception/conf
  FILES_MATCHING PATTERN "*.pb.txt")
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/perception/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/perception/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
