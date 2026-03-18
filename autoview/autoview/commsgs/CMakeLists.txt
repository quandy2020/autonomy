# Copyright 2025 The Openbot Authors (duyongquan)
# Proto3 messages aligned with ROS2 common_interfaces (std_msgs, geometry_msgs, sensor_msgs, nav_msgs).

cmake_minimum_required(VERSION 3.16)

find_package(Protobuf REQUIRED)

set(COMMSGS_PROTO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(COMMSGS_GEN_DIR "${CMAKE_BINARY_DIR}/commsgs")

file(GLOB_RECURSE COMMSGS_PROTO_FILES "${COMMSGS_PROTO_ROOT}/*.proto")

set(COMMSGS_PROTO_SRCS)
set(COMMSGS_PROTO_HDRS)
set(COMMSGS_PROTO_RELATIVE)

foreach(PROTO_ABS ${COMMSGS_PROTO_FILES})
  file(RELATIVE_PATH REL "${COMMSGS_PROTO_ROOT}" "${PROTO_ABS}")
  get_filename_component(REL_DIR "${REL}" DIRECTORY)
  get_filename_component(NAME_WE "${PROTO_ABS}" NAME_WE)
  set(GEN_CC "${COMMSGS_GEN_DIR}/${REL_DIR}/${NAME_WE}.pb.cc")
  set(GEN_H  "${COMMSGS_GEN_DIR}/${REL_DIR}/${NAME_WE}.pb.h")
  list(APPEND COMMSGS_PROTO_SRCS "${GEN_CC}")
  list(APPEND COMMSGS_PROTO_HDRS "${GEN_H}")
  list(APPEND COMMSGS_PROTO_RELATIVE "${REL}")
endforeach()

file(MAKE_DIRECTORY "${COMMSGS_GEN_DIR}")

add_custom_command(
  OUTPUT ${COMMSGS_PROTO_SRCS} ${COMMSGS_PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    --proto_path=${COMMSGS_PROTO_ROOT}
    --cpp_out=${COMMSGS_GEN_DIR}
    ${COMMSGS_PROTO_RELATIVE}
  WORKING_DIRECTORY ${COMMSGS_PROTO_ROOT}
  DEPENDS ${COMMSGS_PROTO_FILES}
  COMMENT "Generating C++ from commsgs protos"
)

add_custom_target(commsgs_protos ALL DEPENDS ${COMMSGS_PROTO_SRCS} ${COMMSGS_PROTO_HDRS})

add_library(autoview_commsgs STATIC ${COMMSGS_PROTO_SRCS} ${COMMSGS_PROTO_HDRS})
add_dependencies(autoview_commsgs commsgs_protos)
target_include_directories(autoview_commsgs PUBLIC
  ${COMMSGS_GEN_DIR}
  ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(autoview_commsgs PUBLIC ${Protobuf_LIBRARIES})
