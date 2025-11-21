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


message(STATUS "Building autolink examples")

# Generate protobuf files for examples
set(EXAMPLES_PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/examples.proto)
set(EXAMPLES_PROTO_SRC ${CMAKE_CURRENT_BINARY_DIR}/proto/examples.pb.cc)
set(EXAMPLES_PROTO_HDR ${CMAKE_CURRENT_BINARY_DIR}/proto/examples.pb.h)

# Create proto directory in build
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/proto)

# Generate protobuf files
add_custom_command(
    OUTPUT ${EXAMPLES_PROTO_SRC} ${EXAMPLES_PROTO_HDR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/proto
            --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
            --proto_path=${PROJECT_SOURCE_DIR}/autolink/proto
            ${EXAMPLES_PROTO_FILE}
    DEPENDS ${EXAMPLES_PROTO_FILE}
    COMMENT "Generating protobuf files for examples"
)

# dependencies
set(dependencies 
    autolink
    fastdds
    fastcdr
    pthread
    ${Protobuf_LIBRARIES}
)

# Create custom target to ensure protobuf files are generated before compilation
add_custom_target(examples_protos DEPENDS ${EXAMPLES_PROTO_SRC} ${EXAMPLES_PROTO_HDR})

# talker example
add_executable(autolink.examples.talker talker.cpp ${EXAMPLES_PROTO_SRC})
add_dependencies(autolink.examples.talker examples_protos)
target_include_directories(autolink.examples.talker PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)
target_link_libraries(autolink.examples.talker ${dependencies})


# listener example
add_executable(autolink.examples.listener listener.cpp ${EXAMPLES_PROTO_SRC})
add_dependencies(autolink.examples.listener examples_protos)
target_include_directories(autolink.examples.listener PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)
target_link_libraries(autolink.examples.listener ${dependencies})

# talker_reader example (both writer and reader in one node)
add_executable(autolink.examples.talker_reader talker_reader.cpp ${EXAMPLES_PROTO_SRC})
add_dependencies(autolink.examples.talker_reader examples_protos)
target_include_directories(autolink.examples.talker_reader PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)
target_link_libraries(autolink.examples.talker_reader ${dependencies})


# service example
add_executable(autolink.examples.service service.cpp ${EXAMPLES_PROTO_SRC})
add_dependencies(autolink.examples.service examples_protos)
target_include_directories(autolink.examples.service PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)
target_link_libraries(autolink.examples.service ${dependencies})

# paramserver example
add_executable(autolink.examples.paramserver paramserver.cpp ${EXAMPLES_PROTO_SRC})
add_dependencies(autolink.examples.paramserver examples_protos)
target_include_directories(autolink.examples.paramserver PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)
target_link_libraries(autolink.examples.paramserver ${dependencies})

# common component example
add_subdirectory(common_component)
add_subdirectory(timer_component)
