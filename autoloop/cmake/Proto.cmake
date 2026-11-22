# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

find_package(Protobuf REQUIRED)
if(NOT TARGET protobuf::protoc AND Protobuf_PROTOC_EXECUTABLE)
  add_executable(protobuf::protoc IMPORTED)
  set_target_properties(protobuf::protoc PROPERTIES
    IMPORTED_LOCATION "${Protobuf_PROTOC_EXECUTABLE}")
endif()
set(AUTOLOOP_GEN_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${AUTOLOOP_GEN_DIR})

function(autoloop_proto target)
  cmake_parse_arguments(ARG "" "" "PROTOS" ${ARGN})
  set(_outs)
  foreach(_p ${ARG_PROTOS})
    file(RELATIVE_PATH _rel ${AUTOLOOP_SRC_ROOT} ${_p})
    get_filename_component(_reldir ${_rel} DIRECTORY)
    get_filename_component(_name ${_p} NAME_WE)
    set(_cc "${AUTOLOOP_GEN_DIR}/${_reldir}/${_name}.pb.cc")
    set(_h "${AUTOLOOP_GEN_DIR}/${_reldir}/${_name}.pb.h")
    add_custom_command(
      OUTPUT ${_cc} ${_h}
      COMMAND protobuf::protoc
              --cpp_out=${AUTOLOOP_GEN_DIR}
              --proto_path=${AUTOLOOP_SRC_ROOT}
              ${_p}
      DEPENDS ${_p}
      COMMENT "Generating C++ protocol buffer ${_rel}")
    list(APPEND _outs ${_cc} ${_h})
  endforeach()
  add_library(${target} ${_outs})
  target_include_directories(${target} PUBLIC ${AUTOLOOP_GEN_DIR} ${Protobuf_INCLUDE_DIRS})
  if(TARGET protobuf::libprotobuf)
    target_link_libraries(${target} PUBLIC protobuf::libprotobuf)
  else()
    target_link_libraries(${target} PUBLIC ${Protobuf_LIBRARIES})
  endif()
  target_compile_features(${target} PUBLIC cxx_std_20)
endfunction()
