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

cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(generating_custom_msgs VERSION 1.0.0)

find_package(Protobuf REQUIRED)

# Proto include from main automsgs build (set by proto/CMakeLists.txt)
if(NOT DEFINED AUTOMSGS_PROTO_INCLUDE_DIR)
  message(FATAL_ERROR "AUTOMSGS_PROTO_INCLUDE_DIR not set. Build from automsgs root with add_subdirectory(examples).")
endif()

set(CUSTOM_PROTO_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/proto_include)
set(CUSTOM_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
set(CUSTOM_PROTO_STAMP ${CUSTOM_PROTO_INCLUDE}/.stamp)

file(GLOB CUSTOM_PROTO_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/proto/automsgs/custom_msgs/*.proto")

add_custom_command(
  OUTPUT ${CUSTOM_PROTO_STAMP}
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/proto ${CUSTOM_PROTO_INCLUDE}
  COMMAND ${CMAKE_COMMAND} -E touch ${CUSTOM_PROTO_STAMP}
  DEPENDS ${CUSTOM_PROTO_SOURCES}
  COMMENT "Copy custom proto tree for protoc"
  VERBATIM
)

set(CUSTOM_GEN_SOURCES "")
set(CUSTOM_GEN_HEADERS "")
foreach(PROTO_FILE ${CUSTOM_PROTO_SOURCES})
  get_filename_component(PROTO_WE ${PROTO_FILE} NAME_WE)
  get_filename_component(PROTO_DIR ${PROTO_FILE} DIRECTORY)
  file(RELATIVE_PATH REL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto ${PROTO_DIR})
  set(REL_PATH "${REL_DIR}/${PROTO_WE}")
  set(OUT_CC "${CUSTOM_GEN_DIR}/${REL_PATH}.pb.cc")
  set(OUT_H  "${CUSTOM_GEN_DIR}/${REL_PATH}.pb.h")
  list(APPEND CUSTOM_GEN_SOURCES ${OUT_CC})
  list(APPEND CUSTOM_GEN_HEADERS ${OUT_H})
  add_custom_command(
    OUTPUT ${OUT_CC} ${OUT_H}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CUSTOM_GEN_DIR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS -I${AUTOMSGS_PROTO_INCLUDE_DIR} -I${CUSTOM_PROTO_INCLUDE}
         --cpp_out=${CUSTOM_GEN_DIR}
         ${CUSTOM_PROTO_INCLUDE}/${REL_PATH}.proto
    DEPENDS ${CUSTOM_PROTO_STAMP} ${CUSTOM_PROTO_SOURCES}
    COMMENT "Generate C++ from ${PROTO_WE}.proto"
    VERBATIM
  )
endforeach()

add_custom_target(generating_custom_msgs_proto_copy ALL DEPENDS ${CUSTOM_PROTO_STAMP})

add_library(${PROJECT_NAME}-msgs SHARED ${CUSTOM_GEN_SOURCES})
target_include_directories(${PROJECT_NAME}-msgs PUBLIC
  ${CUSTOM_GEN_DIR}
  ${AUTOMSGS_PROTO_GEN_DIR}
)
target_link_libraries(${PROJECT_NAME}-msgs PUBLIC protobuf::libprotobuf)
add_dependencies(${PROJECT_NAME}-msgs generating_custom_msgs_proto_copy automsgs_proto)
set_target_properties(${PROJECT_NAME}-msgs PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}-msgs automsgs_proto)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
