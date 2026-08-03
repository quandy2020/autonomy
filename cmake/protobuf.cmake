# Generate C++ sources from autonomy/*.proto (and optional gRPC services).
#
# Sets / appends:
#   ALL_PROTO_SRCS, ALL_PROTO_HDRS
#   ALL_GRPC_SERVICE_SRCS, ALL_GRPC_SERVICE_HDRS (when BUILD_GRPC)
#   ALL_LIBRARY_SRCS / ALL_LIBRARY_HDRS

set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
set(ALL_GRPC_SERVICE_SRCS)
set(ALL_GRPC_SERVICE_HDRS)

set(PROTOBUF_PROTOC_EXECUTABLE "${Protobuf_PROTOC_EXECUTABLE}")

set(_AUTONOMY_PROTO_INCLUDES -I ${PROJECT_SOURCE_DIR})
if(DEFINED AUTOMSGS_PROTO_INCLUDE_DIR)
  list(APPEND _AUTONOMY_PROTO_INCLUDES -I ${AUTOMSGS_PROTO_INCLUDE_DIR})
endif()
set(_AUTONOMY_PROTO_DEPS "")
if(TARGET automsgs_proto_copy)
  list(APPEND _AUTONOMY_PROTO_DEPS automsgs_proto_copy)
endif()

file(GLOB_RECURSE ALL_PROTOS "${PROJECT_SOURCE_DIR}/autonomy/*.proto")
list(FILTER ALL_PROTOS EXCLUDE REGEX ".*/commsgs/proto/.*")
file(GLOB_RECURSE ALL_GRPC_SERVICES
  "${PROJECT_SOURCE_DIR}/autonomy/*_service.proto")

# Service protos are handled by the gRPC pass (or skipped entirely).
if(ALL_GRPC_SERVICES)
  list(REMOVE_ITEM ALL_PROTOS ${ALL_GRPC_SERVICES})
endif()

if(BUILD_GRPC)
  find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
  if(NOT GRPC_CPP_PLUGIN)
    message(FATAL_ERROR
      "grpc_cpp_plugin not found. Install gRPC and ensure it is on PATH.")
  endif()

  foreach(ABS_FIL ${ALL_GRPC_SERVICES})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    set(_grpc_cc "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc")
    set(_grpc_h  "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h")
    set(_pb_cc   "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    set(_pb_h    "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

    list(APPEND ALL_GRPC_SERVICE_SRCS ${_grpc_cc} ${_pb_cc})
    list(APPEND ALL_GRPC_SERVICE_HDRS ${_grpc_h} ${_pb_h})

    add_custom_command(
      OUTPUT ${_grpc_cc} ${_grpc_h} ${_pb_cc} ${_pb_h}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS
        --grpc_out=${PROJECT_BINARY_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --cpp_out=${PROJECT_BINARY_DIR}
        ${_AUTONOMY_PROTO_INCLUDES}
        ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${_AUTONOMY_PROTO_DEPS}
      COMMENT "Running gRPC/protoc on ${REL_FIL}"
      VERBATIM
    )
  endforeach()

  set_source_files_properties(
    ${ALL_GRPC_SERVICE_SRCS} ${ALL_GRPC_SERVICE_HDRS}
    PROPERTIES GENERATED TRUE)
  list(APPEND ALL_LIBRARY_HDRS ${ALL_GRPC_SERVICE_HDRS})
  list(APPEND ALL_LIBRARY_SRCS ${ALL_GRPC_SERVICE_SRCS})
endif()

foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  set(_pb_cc "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  set(_pb_h  "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  list(APPEND ALL_PROTO_SRCS ${_pb_cc})
  list(APPEND ALL_PROTO_HDRS ${_pb_h})

  add_custom_command(
    OUTPUT ${_pb_cc} ${_pb_h}
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROJECT_BINARY_DIR} ${_AUTONOMY_PROTO_INCLUDES} ${ABS_FIL}
    DEPENDS ${ABS_FIL} ${_AUTONOMY_PROTO_DEPS}
    COMMENT "Running protoc on ${REL_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(
  ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})
