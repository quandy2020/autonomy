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

# Get proto source from parent directory (defined in parent CMakeLists.txt)
set(EXAMPLES_PROTO_SRC ${CMAKE_CURRENT_BINARY_DIR}/../proto/examples.pb.cc)

# dependencies
set(common_component_dependencies 
    autolink
    fastdds
    fastcdr
    pthread
    ${Protobuf_LIBRARIES}
)

add_executable(channel_test_writer
  channel_test_writer.cpp
  ${EXAMPLES_PROTO_SRC}
)

add_dependencies(channel_test_writer examples_protos)

target_include_directories(channel_test_writer PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}/../proto
    ${CMAKE_CURRENT_SOURCE_DIR}/../proto
)

target_link_libraries(channel_test_writer
  ${common_component_dependencies}
)

add_executable(channel_prediction_writer
  channel_prediction_writer.cpp
  ${EXAMPLES_PROTO_SRC}
)

add_dependencies(channel_prediction_writer examples_protos)

target_include_directories(channel_prediction_writer PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}/../proto
    ${CMAKE_CURRENT_SOURCE_DIR}/../proto
)

target_link_libraries(channel_prediction_writer
  ${common_component_dependencies}
)

add_library(common_component_example SHARED
  common_component_example.cpp
)

add_dependencies(common_component_example examples_protos)

target_include_directories(common_component_example PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}/../proto
    ${CMAKE_CURRENT_SOURCE_DIR}/../proto
)

target_link_libraries(common_component_example
  ${common_component_dependencies}
)

set_target_properties(common_component_example PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/autolink/examples/common_component_example/"
)

file(COPY common.dag common.launch
  DESTINATION ${CMAKE_BINARY_DIR}/share/examples/common_component_example
)

install(FILES common.dag common.launch
  DESTINATION share/examples/common_component_example
)

install(TARGETS common_component_example
  LIBRARY DESTINATION lib
)

