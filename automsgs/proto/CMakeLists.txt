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

# Proto include layout: copy msgs/, srvs/, rpcs/, actions/ to proto_include/automsgs/
# so that import "automsgs/msgs/..." works with -I proto_include.
#-----------------------------------------------------------------------------
set(AM_PROTO_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_include)
set(AM_PROTO_STAMP ${AM_PROTO_INCLUDE_DIR}/.stamp)

file(GLOB_RECURSE AM_ALL_PROTO_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/msgs/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/srvs/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/rpcs/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/actions/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/task/*.proto")

add_custom_command(
  OUTPUT ${AM_PROTO_STAMP}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AM_PROTO_INCLUDE_DIR}/automsgs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/msgs ${AM_PROTO_INCLUDE_DIR}/automsgs/msgs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/srvs ${AM_PROTO_INCLUDE_DIR}/automsgs/srvs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/rpcs ${AM_PROTO_INCLUDE_DIR}/automsgs/rpcs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/actions ${AM_PROTO_INCLUDE_DIR}/automsgs/actions
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/task ${AM_PROTO_INCLUDE_DIR}/automsgs/task
  COMMAND ${CMAKE_COMMAND} -E touch ${AM_PROTO_STAMP}
  DEPENDS ${AM_ALL_PROTO_SOURCES}
  COMMENT "Copy proto tree to ${AM_PROTO_INCLUDE_DIR}/automsgs for protoc -I"
  VERBATIM
)

include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/msgs_protoc.cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/msgs_generate.cmake")

set(AM_GEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen")
set(AM_MSGS_GEN_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../tools/automsgs_msgs_generate.py")

# One list of protos per directory. The package name is that path with dots.
set(_proto_dirs "")
foreach(_proto ${AM_ALL_PROTO_SOURCES})
  cmake_path(GET _proto PARENT_PATH _dir)
  cmake_path(RELATIVE_PATH _dir BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_VARIABLE _rel)
  string(REPLACE "/" "_" _key "${_rel}")
  list(APPEND "_protos_${_key}" "${_proto}")
  list(APPEND _proto_dirs "${_rel}")
endforeach()
list(REMOVE_DUPLICATES _proto_dirs)

set(AM_ALL_SOURCES "")
foreach(_rel ${_proto_dirs})
  string(REPLACE "/" "_" _key "${_rel}")
  string(REPLACE "/" "." _pkg "automsgs.${_rel}")
  set(_sources "")
  set(_headers "")
  set(_details "")
  set(_python "")
  automsgs_msgs_generate_messages_impl(
    PYTHON_INTERPRETER Python3::Interpreter
    MSGS_GEN_SCRIPT "${AM_MSGS_GEN_SCRIPT}"
    PROTOC_EXEC protobuf::protoc
    PROTOC_PLUGIN ""
    PROTO_PACKAGE "${_pkg}"
    PROTO_PATH "${AM_PROTO_INCLUDE_DIR}"
    PROTO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
    PROTO_STAMP "${AM_PROTO_STAMP}"
    INPUT_PROTOS ${_protos_${_key}}
    OUTPUT_DIRECTORY "${AM_GEN_OUTPUT_DIR}"
    OUTPUT_SOURCES _sources
    OUTPUT_HEADERS _headers
    OUTPUT_DETAIL_HEADERS _details
    OUTPUT_PYTHON _python)
  list(APPEND AM_ALL_SOURCES ${_sources})
endforeach()

# Same directory as the protoc rules, so the generated sources are real inputs.
autocmake_library(automsgs SHARED
  SOURCES ${AM_ALL_SOURCES}
  DEPENDENCIES protobuf::libprotobuf)
target_include_directories(automsgs PUBLIC
  "$<BUILD_INTERFACE:${AM_GEN_OUTPUT_DIR}>"
  "$<BUILD_INTERFACE:${AM_PROTO_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_compile_definitions(automsgs PRIVATE automsgs_core_EXPORTS)
add_library(automsgs_core ALIAS automsgs)
add_library(automsgs_proto ALIAS automsgs)

add_custom_target(automsgs_proto_copy ALL DEPENDS ${AM_PROTO_STAMP})
add_dependencies(automsgs automsgs_proto_copy)

install(DIRECTORY ${AM_GEN_OUTPUT_DIR}/automsgs
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hh")
foreach(_tree IN ITEMS msgs srvs rpcs actions task)
  install(DIRECTORY ${_tree}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs/proto
    FILES_MATCHING PATTERN "*.proto")
endforeach()

# Install generated Python modules (layout: gen/python/automsgs/msgs/..., srvs/...)
install(DIRECTORY ${AM_GEN_OUTPUT_DIR}/python/automsgs
  DESTINATION ${AUTOMSGS_PYTHON_INSTALL_PATH}
  FILES_MATCHING PATTERN "*.py")

# Export for examples (e.g. generating_custom_msgs)
set(AUTOMSGS_PROTO_INCLUDE_DIR ${AM_PROTO_INCLUDE_DIR} PARENT_SCOPE)
set(AUTOMSGS_PROTO_GEN_DIR ${AM_GEN_OUTPUT_DIR} PARENT_SCOPE)
