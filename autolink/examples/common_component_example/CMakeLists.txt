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

if(NOT TARGET autolink)
  return()
endif()
if(NOT TARGET examples_proto)
  return()
endif()

# Component 示例：构建为动态库，供 mainboard 按 common.dag 加载
add_library(common_component_example SHARED
  common_component_example.cpp
)

target_include_directories(common_component_example
  PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${EXAMPLES_PROTO_GEN_DIR}
)

target_link_libraries(common_component_example
  PRIVATE
    autolink
    examples_proto
)

# 输出为 libcommon_component_example.so，放到 lib/examples/common_component_example
set_target_properties(common_component_example PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/examples/common_component_example"
)
