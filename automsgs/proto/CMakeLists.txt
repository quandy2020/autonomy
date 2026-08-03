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

#-----------------------------------------------------------------------------
# Dependencies
#-----------------------------------------------------------------------------
autonomy_require_protobuf()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

#-----------------------------------------------------------------------------
# Proto include layout: copy msgs/, srvs/, rpcs/, actions/ to proto_include/automsgs/
# so that import "automsgs/msgs/..." works with -I proto_include.
#-----------------------------------------------------------------------------
set(AM_PROTO_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_include)
set(AM_PROTO_STAMP ${AM_PROTO_INCLUDE_DIR}/.stamp)

file(GLOB_RECURSE AM_ALL_PROTO_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/msgs/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/srvs/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/rpcs/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/actions/*.proto")

add_custom_command(
  OUTPUT ${AM_PROTO_STAMP}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${AM_PROTO_INCLUDE_DIR}/automsgs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/msgs ${AM_PROTO_INCLUDE_DIR}/automsgs/msgs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/srvs ${AM_PROTO_INCLUDE_DIR}/automsgs/srvs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/rpcs ${AM_PROTO_INCLUDE_DIR}/automsgs/rpcs
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/actions ${AM_PROTO_INCLUDE_DIR}/automsgs/actions
  COMMAND ${CMAKE_COMMAND} -E touch ${AM_PROTO_STAMP}
  DEPENDS ${AM_ALL_PROTO_SOURCES}
  COMMENT "Copy proto tree to ${AM_PROTO_INCLUDE_DIR}/automsgs for protoc -I"
  VERBATIM
)

#-----------------------------------------------------------------------------
# Discover packages (one per directory that contains .proto)
#-----------------------------------------------------------------------------
set(AM_PKG_DIRS "")
foreach(proto ${AM_ALL_PROTO_SOURCES})
  get_filename_component(dir ${proto} DIRECTORY)
  file(RELATIVE_PATH reldir ${CMAKE_CURRENT_SOURCE_DIR} ${dir})
  list(APPEND AM_PKG_DIRS ${reldir})
endforeach()
list(REMOVE_DUPLICATES AM_PKG_DIRS)

#-----------------------------------------------------------------------------
# Include generation cmake (order: string_utils, protoc, generate; factory optional)
#-----------------------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/msgs_string_utils.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/msgs_protoc.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/msgs_generate.cmake)

set(AM_GEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
set(AM_MSGS_GEN_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../tools/automsgs_msgs_generate.py)

set(AM_ALL_SOURCES "")
set(AM_ALL_HEADERS "")
set(AM_ALL_DETAIL_HEADERS "")
set(AM_ALL_PYTHON "")

foreach(pkg_dir ${AM_PKG_DIRS})
  # Package name: automsgs.msgs.std_msgs, automsgs.srvs, etc.
  string(REPLACE "/" "." pkg "automsgs.${pkg_dir}")

  set(pkg_protos "")
  foreach(proto ${AM_ALL_PROTO_SOURCES})
    get_filename_component(d ${proto} DIRECTORY)
    file(RELATIVE_PATH rd ${CMAKE_CURRENT_SOURCE_DIR} ${d})
    if(rd STREQUAL pkg_dir)
      list(APPEND pkg_protos ${proto})
    endif()
  endforeach()

  if(NOT pkg_protos)
    continue()
  endif()

  set(pkg_sources "")
  set(pkg_headers "")
  set(pkg_detail_headers "")
  set(pkg_python "")

  automsgs_msgs_generate_messages_impl(
    PYTHON_INTERPRETER  Python3::Interpreter
    MSGS_GEN_SCRIPT     ${AM_MSGS_GEN_SCRIPT}
    PROTOC_EXEC         protobuf::protoc
    GZ_PROTOC_PLUGIN    ""
    PROTO_PACKAGE       ${pkg}
    PROTO_PATH          ${AM_PROTO_INCLUDE_DIR}
    PROTO_SOURCE_DIR    ${CMAKE_CURRENT_SOURCE_DIR}
    PROTO_STAMP         ${AM_PROTO_STAMP}
    INPUT_PROTOS        ${pkg_protos}
    OUTPUT_DIRECTORY    ${AM_GEN_OUTPUT_DIR}
    OUTPUT_SOURCES      pkg_sources
    OUTPUT_HEADERS      pkg_headers
    OUTPUT_DETAIL_HEADERS pkg_detail_headers
    OUTPUT_PYTHON       pkg_python
  )

  list(APPEND AM_ALL_SOURCES ${pkg_sources})
  list(APPEND AM_ALL_HEADERS ${pkg_headers})
  list(APPEND AM_ALL_DETAIL_HEADERS ${pkg_detail_headers})
  list(APPEND AM_ALL_PYTHON ${pkg_python})
endforeach()

#-----------------------------------------------------------------------------
# Library from generated .pb.cc
#-----------------------------------------------------------------------------
add_library(automsgs_proto_obj OBJECT ${AM_ALL_SOURCES})
target_include_directories(automsgs_proto_obj PUBLIC
  "$<BUILD_INTERFACE:${AM_GEN_OUTPUT_DIR}>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(automsgs_proto_obj PUBLIC protobuf::libprotobuf)
set_target_properties(automsgs_proto_obj PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# Ensure stamp is built before any generated source
add_custom_target(automsgs_proto_copy ALL DEPENDS ${AM_PROTO_STAMP})
add_dependencies(automsgs_proto_obj automsgs_proto_copy)

#-----------------------------------------------------------------------------
# Install generated headers & proto tree (no standalone libautomsgs_proto.so)
#-----------------------------------------------------------------------------
include(GNUInstallDirs)
install(DIRECTORY ${AM_GEN_OUTPUT_DIR}/automsgs
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hh")

install(
  DIRECTORY msgs
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs/proto
  FILES_MATCHING PATTERN "*.proto"
)
install(
  DIRECTORY srvs
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs/proto
  FILES_MATCHING PATTERN "*.proto"
)
install(
  DIRECTORY rpcs
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs/proto
  FILES_MATCHING PATTERN "*.proto"
)
install(
  DIRECTORY actions
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs/proto
  FILES_MATCHING PATTERN "*.proto"
)

# Install generated Python modules (layout: gen/python/automsgs/msgs/..., srvs/...)
install(DIRECTORY ${AM_GEN_OUTPUT_DIR}/python/automsgs
  DESTINATION ${AUTOMSGS_PYTHON_INSTALL_PATH}
  FILES_MATCHING PATTERN "*.py")

# Export for examples (e.g. generating_custom_msgs)
set(AUTOMSGS_PROTO_INCLUDE_DIR ${AM_PROTO_INCLUDE_DIR} PARENT_SCOPE)
set(AUTOMSGS_PROTO_GEN_DIR ${AM_GEN_OUTPUT_DIR} PARENT_SCOPE)
