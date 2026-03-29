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

set(EXAMPLES_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin/examples")
set(EXAMPLES_PROTO_GEN_DIR "${CMAKE_BINARY_DIR}/examples_proto")
set(EXAMPLES_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(MAKE_DIRECTORY "${EXAMPLES_PROTO_GEN_DIR}")

# protoc 3.x 将 .pb.cc/.pb.h 生成到 cpp_out 根目录（不按 package 建子目录）
set(EXAMPLES_PROTO_SRC "${EXAMPLES_PROTO_GEN_DIR}/examples.pb.cc")
set(EXAMPLES_PROTO_HDR "${EXAMPLES_PROTO_GEN_DIR}/examples.pb.h")
add_custom_command(
  OUTPUT ${EXAMPLES_PROTO_SRC} ${EXAMPLES_PROTO_HDR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${EXAMPLES_PROTO_GEN_DIR}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    --proto_path=${EXAMPLES_PROTO_DIR}
    --cpp_out=${EXAMPLES_PROTO_GEN_DIR}
    examples.proto
  WORKING_DIRECTORY ${EXAMPLES_PROTO_DIR}
  DEPENDS ${EXAMPLES_PROTO_DIR}/examples.proto
  COMMENT "Generating C++ for examples.proto"
  VERBATIM
)
# OBJECT 库，不产出 libexamples_proto.a；.o 直接链入示例可执行文件与 common_component_example.so
add_library(examples_proto OBJECT ${EXAMPLES_PROTO_SRC} ${EXAMPLES_PROTO_HDR})
set_target_properties(examples_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(examples_proto PUBLIC ${EXAMPLES_PROTO_GEN_DIR} ${Protobuf_INCLUDE_DIRS})
target_link_libraries(examples_proto PUBLIC ${Protobuf_LIBRARIES})

function(autolink_example NAME SRCS)
  add_executable(${NAME} ${SRCS})
  target_include_directories(${NAME} PRIVATE ${CMAKE_SOURCE_DIR} ${EXAMPLES_PROTO_GEN_DIR})
  target_link_libraries(${NAME} PRIVATE autolink)
  set_target_properties(${NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${EXAMPLES_OUTPUT_DIR}")
endfunction()

function(autolink_example_with_proto NAME SRCS)
  autolink_example(${NAME} ${SRCS})
  target_link_libraries(${NAME} PRIVATE examples_proto)
endfunction()

# 依赖 examples proto 的示例
autolink_example_with_proto(autolink_example_talker talker.cpp)
autolink_example_with_proto(autolink_example_listener listener.cpp)
autolink_example_with_proto(autolink_example_talker_listener talker_listener.cpp)
autolink_example_with_proto(autolink_example_service service.cpp)
autolink_example_with_proto(autolink_example_paramserver paramserver.cpp)
autolink_example_with_proto(autolink_example_record record.cpp)
autolink_example_with_proto(autolink_example_action_listener action_listener.cpp)
autolink_example_with_proto(autolink_example_action_talker action_talker.cpp)
autolink_example_with_proto(autolink_example_action_talker2 action_talker2.cpp)

# component
add_subdirectory(timer_component_example)
add_subdirectory(common_component_example)