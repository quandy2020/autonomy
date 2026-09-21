# Copyright 2026 The Openbot Authors
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

# Atla2 protobuf definitions + optional codegen against automsgs.

file(GLOB_RECURSE _ATLA2_PROTOS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

message(STATUS "atla2 proto sources: ${CMAKE_CURRENT_SOURCE_DIR}")

# Resolve automsgs include if parent did not export it.
if(NOT AUTOMSGS_PROTO_INCLUDE_DIR)
  if(DEFINED CACHE{AUTOMSGS_PROTO_INCLUDE_DIR})
    set(AUTOMSGS_PROTO_INCLUDE_DIR "${AUTOMSGS_PROTO_INCLUDE_DIR}")
  elseif(EXISTS
      "${CMAKE_BINARY_DIR}/autonomy/automsgs/proto/proto_include/automsgs")
    set(AUTOMSGS_PROTO_INCLUDE_DIR
      "${CMAKE_BINARY_DIR}/autonomy/automsgs/proto/proto_include")
  elseif(EXISTS
      "${PROJECT_BINARY_DIR}/autonomy/automsgs/proto/proto_include/automsgs")
    set(AUTOMSGS_PROTO_INCLUDE_DIR
      "${PROJECT_BINARY_DIR}/autonomy/automsgs/proto/proto_include")
  endif()
endif()

# Default ON when include + protoc + automsgs target are available.
if(NOT DEFINED ATLA2_GENERATE_PROTO)
  if(AUTOMSGS_PROTO_INCLUDE_DIR AND TARGET automsgs_proto)
    option(ATLA2_GENERATE_PROTO "Generate C++ from atla2/*.proto" ON)
  else()
    option(ATLA2_GENERATE_PROTO "Generate C++ from atla2/*.proto" OFF)
  endif()
endif()

if(ATLA2_GENERATE_PROTO)
  if(NOT AUTOMSGS_PROTO_INCLUDE_DIR)
    message(FATAL_ERROR
      "ATLA2_GENERATE_PROTO=ON requires AUTOMSGS_PROTO_INCLUDE_DIR")
  endif()
  find_package(Protobuf REQUIRED)
  set(_ATLA2_PROTO_GEN "${CMAKE_CURRENT_BINARY_DIR}/gen")
  file(MAKE_DIRECTORY ${_ATLA2_PROTO_GEN})
  set(_ATLA2_PROTO_SRCS "")
  foreach(_p IN LISTS _ATLA2_PROTOS)
    file(RELATIVE_PATH _rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_p}")
    get_filename_component(_rel_dir "${_rel}" DIRECTORY)
    get_filename_component(_rel_we "${_rel}" NAME_WE)
    if(_rel_dir STREQUAL "")
      set(_out_cc "${_ATLA2_PROTO_GEN}/${_rel_we}.pb.cc")
      set(_out_h  "${_ATLA2_PROTO_GEN}/${_rel_we}.pb.h")
    else()
      set(_out_cc "${_ATLA2_PROTO_GEN}/${_rel_dir}/${_rel_we}.pb.cc")
      set(_out_h  "${_ATLA2_PROTO_GEN}/${_rel_dir}/${_rel_we}.pb.h")
    endif()
    add_custom_command(
      OUTPUT ${_out_cc} ${_out_h}
      COMMAND ${CMAKE_COMMAND} -E make_directory
              "${_ATLA2_PROTO_GEN}/${_rel_dir}"
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS -I${CMAKE_CURRENT_SOURCE_DIR}
           -I${AUTOMSGS_PROTO_INCLUDE_DIR}
           --cpp_out=${_ATLA2_PROTO_GEN}
           ${_p}
      DEPENDS ${_p}
      COMMENT "atla2 protoc ${_rel}"
      VERBATIM)
    list(APPEND _ATLA2_PROTO_SRCS ${_out_cc})
  endforeach()

  add_library(atla2_proto ${_ATLA2_PROTO_SRCS})
  add_library(autonomy::atla2_proto ALIAS atla2_proto)
  set_target_properties(atla2_proto PROPERTIES
    OUTPUT_NAME "autonomy_atla2_proto"
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH "\$ORIGIN:${CMAKE_INSTALL_PREFIX}/lib")
  target_include_directories(atla2_proto PUBLIC
    $<BUILD_INTERFACE:${_ATLA2_PROTO_GEN}>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(atla2_proto PUBLIC ATLA2_HAS_PROTO=1)
  target_link_libraries(atla2_proto PUBLIC protobuf::libprotobuf)
  if(TARGET automsgs_proto)
    target_link_libraries(atla2_proto PUBLIC automsgs_proto)
  elseif(TARGET automsgs)
    target_link_libraries(atla2_proto PUBLIC automsgs)
  endif()

  # Required by autonomy_localization's PUBLIC link → must be in AutonomyExport.
  set_property(GLOBAL APPEND PROPERTY AUTONOMY_MODULE_TARGETS atla2_proto)

  if(TARGET autonomy_localization)
    target_link_libraries(autonomy_localization PUBLIC atla2_proto)
    target_compile_definitions(autonomy_localization PUBLIC ATLA2_HAS_PROTO=1)
  endif()

  install(
    DIRECTORY ${_ATLA2_PROTO_GEN}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.pb.h")

  message(STATUS "atla2: protobuf codegen ENABLED -> ${_ATLA2_PROTO_GEN}")
else()
  message(STATUS
    "atla2: protobuf codegen OFF (set -DATLA2_GENERATE_PROTO=ON when "
    "AUTOMSGS_PROTO_INCLUDE_DIR is available)")
endif()

# Always install .proto sources for offline tooling.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION share/autonomy/localization/atla2/proto
  FILES_MATCHING PATTERN "*.proto" PATTERN "*.md"
)
