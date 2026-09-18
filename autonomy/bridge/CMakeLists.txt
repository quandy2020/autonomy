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

# autonomy/bridge — onboard gRPC external API (skipped if BUILD_GRPC=OFF)

include(autonomy_module)
autonomy_module(bridge)

# Library sources: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_BRIDGE_SRCS)
autonomy_library(
  DEPENDENCIES autonomy_common autonomy_system autonomy_task
  FEATURES grpc
  SRCS ${_BRIDGE_SRCS}
)
# GCC 11 ICE (RTL pass "ree") on heavily-templated action client instantiations.
# Disable that pass for this TU only.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set_source_files_properties(
    "${CMAKE_CURRENT_SOURCE_DIR}/node_client.cpp"
    PROPERTIES COMPILE_OPTIONS "-fno-ree")
endif()
unset(_BRIDGE_SRCS)

# Process entrypoint: autonomy.bridge
autonomy_binary(autonomy.bridge SRCS bridge_main.cpp)

# Runtime assets → share/autonomy/bridge/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/bridge/conf
  FILES_MATCHING PATTERN "*.pb.txt")
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/bridge/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/bridge/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
