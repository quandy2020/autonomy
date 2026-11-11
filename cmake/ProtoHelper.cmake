# Copyright 2025 The Openbot Authors (duyongquan)
#
# -----------------------------------------------------------------------------
# Proto helper — domain .proto → autonomy_proto
#
# Included by Functions.cmake. Generates C++ sources for every .proto under
# AUTONOMY_ENABLED_MODULES and packs them into shared target autonomy_proto
# (or an INTERFACE stub when no protos are present).
#
# Rules
#   *_service.proto   gRPC + pb when BUILD_GRPC=ON (needs grpc_cpp_plugin)
#   other .proto      plain protoc --cpp_out
#   commsgs/proto/*   skipped (owned by automsgs)
#
# Include paths
#   -I ${PROJECT_SOURCE_DIR}
#   -I ${AUTOMSGS_PROTO_INCLUDE_DIR}   when set by embedded automsgs
#
# Public API
#   autonomy_add_proto()
# -----------------------------------------------------------------------------

include_guard(GLOBAL)

# --- single file -------------------------------------------------------------
# Run protoc (and optionally the gRPC plugin) for one .proto.
#
#   abs         absolute path to the .proto
#   grpc_out    TRUE → emit *.grpc.pb.{cc,h} + *.pb.{cc,h}
#               FALSE → emit *.pb.{cc,h} only
#
# Appends generated paths to _AUTONOMY_PROTO_SRCS / _AUTONOMY_PROTO_HDRS in
# the parent scope. Output layout mirrors the source tree under
# ${PROJECT_BINARY_DIR}.
function(_autonomy_protoc abs grpc_out)
  file(RELATIVE_PATH rel "${PROJECT_SOURCE_DIR}" "${abs}")
  get_filename_component(dir "${rel}" DIRECTORY)
  get_filename_component(we "${rel}" NAME_WE)
  set(base "${PROJECT_BINARY_DIR}/${dir}/${we}")

  if(grpc_out)
    set(outs "${base}.grpc.pb.cc" "${base}.grpc.pb.h" "${base}.pb.cc" "${base}.pb.h")
    set(srcs "${base}.grpc.pb.cc" "${base}.pb.cc")
    set(hdrs "${base}.grpc.pb.h" "${base}.pb.h")
    add_custom_command(OUTPUT ${outs}
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --grpc_out=${PROJECT_BINARY_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --cpp_out=${PROJECT_BINARY_DIR}
        ${_AUTONOMY_PROTO_INCS} ${abs}
      DEPENDS ${abs} ${_AUTONOMY_PROTO_DEPS}
      COMMENT "gRPC ${rel}" VERBATIM)
  else()
    set(outs "${base}.pb.cc" "${base}.pb.h")
    set(srcs "${base}.pb.cc")
    set(hdrs "${base}.pb.h")
    add_custom_command(OUTPUT ${outs}
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --cpp_out=${PROJECT_BINARY_DIR}
        ${_AUTONOMY_PROTO_INCS} ${abs}
      DEPENDS ${abs} ${_AUTONOMY_PROTO_DEPS}
      COMMENT "protoc ${rel}" VERBATIM)
  endif()

  set(_AUTONOMY_PROTO_SRCS ${_AUTONOMY_PROTO_SRCS} ${srcs} PARENT_SCOPE)
  set(_AUTONOMY_PROTO_HDRS ${_AUTONOMY_PROTO_HDRS} ${hdrs} PARENT_SCOPE)
endfunction()

# --- library -----------------------------------------------------------------
# Collect protos from enabled domains, generate sources, build autonomy_proto.
#
# Targets
#   autonomy_proto / autonomy::proto
#     SHARED when there are sources, else INTERFACE
#     NO_EXPORT — installed with the umbrella, not a separate export set
#
# Link deps (when present)
#   automsgs, protobuf::libprotobuf, grpc++ / grpc
macro(autonomy_add_proto)
  set(_AUTONOMY_PROTO_SRCS "")
  set(_AUTONOMY_PROTO_HDRS "")
  set(_AUTONOMY_PROTO_INCS -I ${PROJECT_SOURCE_DIR})
  if(DEFINED AUTOMSGS_PROTO_INCLUDE_DIR)
    list(APPEND _AUTONOMY_PROTO_INCS -I ${AUTOMSGS_PROTO_INCLUDE_DIR})
  endif()

  # Wait for automsgs to finish copying imported .proto before generating.
  set(_AUTONOMY_PROTO_DEPS "")
  if(TARGET automsgs_proto_copy)
    set(_AUTONOMY_PROTO_DEPS automsgs_proto_copy)
  endif()

  # Gather .proto under enabled domains; drop legacy commsgs copies.
  set(_all "")
  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    file(GLOB_RECURSE _p "${PROJECT_SOURCE_DIR}/autonomy/${_mod}/*.proto")
    list(APPEND _all ${_p})
  endforeach()
  list(FILTER _all EXCLUDE REGEX ".*/commsgs/proto/.*")

  # Split *_service.proto for the gRPC path; remainder stay plain pb.
  set(_grpc ${_all})
  list(FILTER _grpc INCLUDE REGEX "_service\\.proto$")
  if(_grpc)
    list(REMOVE_ITEM _all ${_grpc})
  endif()

  if(BUILD_GRPC AND _grpc)
    find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
    if(NOT GRPC_CPP_PLUGIN)
      message(FATAL_ERROR "grpc_cpp_plugin not found")
    endif()
    foreach(_abs IN LISTS _grpc)
      _autonomy_protoc("${_abs}" TRUE)
    endforeach()
  endif()
  foreach(_abs IN LISTS _all)
    _autonomy_protoc("${_abs}" FALSE)
  endforeach()

  set(_link "")
  if(TARGET automsgs)
    list(APPEND _link automsgs)
  endif()
  if(TARGET protobuf::libprotobuf)
    list(APPEND _link protobuf::libprotobuf)
  endif()
  if(BUILD_GRPC AND _grpc AND TARGET grpc++)
    list(APPEND _link grpc++ grpc)
  endif()

  if(_AUTONOMY_PROTO_SRCS)
    set_source_files_properties(
      ${_AUTONOMY_PROTO_SRCS} ${_AUTONOMY_PROTO_HDRS} PROPERTIES GENERATED TRUE)
    autocmake_library(autonomy_proto SHARED NO_EXPORT
      SOURCES ${_AUTONOMY_PROTO_SRCS} ${_AUTONOMY_PROTO_HDRS}
      INCLUDES ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
      DEPENDENCIES ${_link})
  else()
    autocmake_library(autonomy_proto INTERFACE NO_EXPORT DEPENDENCIES ${_link})
  endif()

  if(NOT TARGET autonomy::proto)
    add_library(autonomy::proto ALIAS autonomy_proto)
  endif()
  if(TARGET automsgs_proto_copy)
    add_dependencies(autonomy_proto automsgs_proto_copy)
  endif()
endmacro()
