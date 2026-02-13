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
#
# Basic demos (channel writer/reader, action server/client, service server/client).
# Uses local proto under basic/proto/ (no dependency on autolink proto path).

message(STATUS "Building autonomy examples (basic)")

# Local proto: basic/proto/examples.proto
set(BASIC_PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/proto/examples.proto")
set(BASIC_EXAMPLES_PROTO_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto")
set(BASIC_EXAMPLES_PB_CC "${BASIC_EXAMPLES_PROTO_DIR}/examples.pb.cc")
set(BASIC_EXAMPLES_PB_H "${BASIC_EXAMPLES_PROTO_DIR}/examples.pb.h")
set_source_files_properties(${BASIC_EXAMPLES_PB_CC} ${BASIC_EXAMPLES_PB_H}
                            PROPERTIES GENERATED TRUE)

add_custom_command(
  OUTPUT ${BASIC_EXAMPLES_PB_CC} ${BASIC_EXAMPLES_PB_H}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${BASIC_EXAMPLES_PROTO_DIR}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
    --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
    proto/examples.proto
  DEPENDS ${BASIC_PROTO_FILE}
  COMMENT "Generating examples.pb from basic/proto/examples.proto"
  VERBATIM
)

add_custom_target(basic_examples_protos
                  DEPENDS ${BASIC_EXAMPLES_PB_CC} ${BASIC_EXAMPLES_PB_H})

# Dependencies: autolink and what it needs
set(BASIC_DEPS autolink ${FastDDS_LIBRARIES} pthread ${Protobuf_LIBRARIES}
               ${GLOG_LIBRARIES})

# ---- Channel demos (use autolink unit_test Chatter) ----
add_executable(autonomy.examples.basic.demo_channel_writer demo_channel_writer.cpp)
add_dependencies(autonomy.examples.basic.demo_channel_writer autolink_protos)
target_include_directories(autonomy.examples.basic.demo_channel_writer PRIVATE ${CMAKE_BINARY_DIR})
target_link_libraries(autonomy.examples.basic.demo_channel_writer ${BASIC_DEPS})

add_executable(autonomy.examples.basic.demo_channel_reader demo_channel_reader.cpp)
add_dependencies(autonomy.examples.basic.demo_channel_reader autolink_protos)
target_include_directories(autonomy.examples.basic.demo_channel_reader PRIVATE ${CMAKE_BINARY_DIR})
target_link_libraries(autonomy.examples.basic.demo_channel_reader ${BASIC_DEPS})

add_executable(autonomy.examples.basic.demo_channel_intra demo_channel_intra.cpp)
add_dependencies(autonomy.examples.basic.demo_channel_intra autolink_protos)
target_include_directories(autonomy.examples.basic.demo_channel_intra PRIVATE ${CMAKE_BINARY_DIR})
target_link_libraries(autonomy.examples.basic.demo_channel_intra ${BASIC_DEPS})

# ---- Action & service demos (use examples.proto) ----
set(BASIC_EXAMPLES_SRCS
    demo_action_server.cpp
    demo_action_client.cpp
    demo_action_intra.cpp
    demo_server_reader.cpp
    demo_server_writer.cpp
    demo_service_intra.cpp
)

foreach(src ${BASIC_EXAMPLES_SRCS})
  get_filename_component(name ${src} NAME_WE)
  add_executable(autonomy.examples.basic.${name} ${src} ${BASIC_EXAMPLES_PB_CC})
  add_dependencies(autonomy.examples.basic.${name} basic_examples_protos)
  target_include_directories(autonomy.examples.basic.${name} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(autonomy.examples.basic.${name} ${BASIC_DEPS})
endforeach()

# SimpleActionServer 进程内 demo（依赖 autonomy）
add_executable(autonomy.examples.basic.demo_action_simple_intra
  demo_action_simple_intra.cpp ${BASIC_EXAMPLES_PB_CC})
add_dependencies(autonomy.examples.basic.demo_action_simple_intra basic_examples_protos)
target_include_directories(autonomy.examples.basic.demo_action_simple_intra
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(autonomy.examples.basic.demo_action_simple_intra ${BASIC_DEPS} autonomy)

# -----------------------------------------------------------------------------
# Component demo 子目录
# -----------------------------------------------------------------------------
add_subdirectory(component)

