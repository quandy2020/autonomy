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

autocmake_binary(automsgs_protoc_plugin
  SOURCES Generator.cc generator_main.cc
  DEPENDENCIES protobuf::libprotoc protobuf::libprotobuf)
target_include_directories(automsgs_protoc_plugin PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${AUTOMSGS_ROOT_DIR}/core/include")
if(UNIX)
  target_link_libraries(automsgs_protoc_plugin PRIVATE pthread)
endif()
