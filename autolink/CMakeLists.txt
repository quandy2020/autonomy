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

cmake_minimum_required(VERSION 3.5)

# Must be set before project() to take effect
set(CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "")
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

project(autolink)
set(TARGET_NAME ${PROJECT_NAME})

# Set CMake policies to suppress warnings
if(POLICY CMP0060)
  cmake_policy(SET CMP0060 NEW)  # Link libraries by full path
endif()

if(POLICY CMP0095)
  cmake_policy(SET CMP0095 NEW)  # RPATH entries are properly escaped
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Configure RPATH - use build RPATH but clean install RPATH
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

# Set RPATH for install - only use install prefix
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib:/usr/local/lib")

# Disable all RPATH-related warnings
set(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION OFF)
set(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION OFF)
set(CMAKE_NO_SYSTEM_FROM_IMPORTED TRUE)
set(CMAKE_PLATFORM_NO_VERSIONED_SONAME TRUE)

# Enable assembly sources for context swap implementations
enable_language(ASM)

# Read version from parent directory's version.json using common function
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/version.cmake")
read_version_from_json(AUTOLINK "${CMAKE_CURRENT_SOURCE_DIR}/../version.json")

# Build options
option(BUILD_TEST "Build autolink unit test" true)
option(BUILD_EXAMPLES "Build autolink examples" true)
option(BUILD_AVIZ "Build aviz" false)
option(BUILD_BENCHMARK "Build benchmark" false)

# Output directories
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include functions
include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_autonomy_project()
google_enable_testing()

# Find dependencies
find_package(FastDDS REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Glog REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Curses REQUIRED)
find_package(absl REQUIRED)

# Find dependencies
pkg_check_modules(UUID REQUIRED uuid)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
pkg_check_modules(Uuid REQUIRED uuid)
pkg_check_modules(Gflags REQUIRED gflags)
pkg_check_modules(NlohmannJson REQUIRED nlohmann_json)
pkg_check_modules(TinyXML2 REQUIRED tinyxml2)


set(AUTOLINK_MSGS_CACHE ON CACHE BOOL "generate autolink proto msgs flag.")

# Generate protobuf files
set(PROTO_SRCS)
set(PROTO_HDRS)
set(PROTO_PY_FILES)
set(PROTO_PY_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/python_autolink)

file(MAKE_DIRECTORY ${PROTO_PY_OUT_DIR})

# Mirror modules under build/autolink so imports like autolink/... resolve
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/autolink)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/modules)
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/modules/
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/autolink)
endif()

# Collect proto inputs: core framework protos from source, modules from build
file(GLOB_RECURSE PROTO_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/autolink/proto/*.proto"
  "${CMAKE_CURRENT_SOURCE_DIR}/autolink/commsgs/*.proto"
  "${CMAKE_CURRENT_BINARY_DIR}/autolink/*.proto"
)

# Generate protobuf source files (preserve relative layout under build tree)
set(PROTO_INPUTS)
foreach(PROTO_FILE ${PROTO_FILES})
  set(REL_BASE "${CMAKE_CURRENT_SOURCE_DIR}")
  if (PROTO_FILE MATCHES "^${CMAKE_CURRENT_BINARY_DIR}/autolink/.*")
    set(REL_BASE "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  file(RELATIVE_PATH REL_FIL ${REL_BASE} ${PROTO_FILE})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${DIR})
  list(APPEND PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc)
  list(APPEND PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h)
  list(APPEND PROTO_PY_FILES
       ${PROTO_PY_OUT_DIR}/${DIR}/${FIL_WE}_pb2.py)
  list(APPEND PROTO_INPUTS ${REL_FIL})
endforeach()

# Create proto directories in build
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/autolink/proto)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/modules)

# Generate protobuf source files
add_custom_command(
    OUTPUT ${PROTO_SRCS} ${PROTO_HDRS} ${PROTO_PY_FILES}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR} 
            --python_out=${PROTO_PY_OUT_DIR} 
            --proto_path=${CMAKE_CURRENT_SOURCE_DIR} 
            --proto_path=${PROJECT_SOURCE_DIR}
            --proto_path=${CMAKE_CURRENT_BINARY_DIR}
            ${PROTO_INPUTS}
    DEPENDS ${PROTO_FILES}
    COMMENT "Generating protobuf files"
)

# Ensure protobufs are generated before building the library
add_custom_target(autolink_protos DEPENDS ${PROTO_SRCS} ${PROTO_HDRS} ${PROTO_PY_FILES})

# Glob all library headers and source files
file(GLOB_RECURSE ALL_LIBRARY_HDRS "autolink/*.hpp" "autolink/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "autolink/*.cpp" "autolink/*.c")
file(GLOB_RECURSE ALL_AVIZ_HDRS "aviz/*.hpp" "aviz/*.h")
file(GLOB_RECURSE ALL_AVIZ_SRCS "aviz/*.cpp" "aviz/*.c")
file(GLOB_RECURSE ALL_BENCHMARK_HDRS "autolink/benchmark/*.hpp" "autolink/benchmark/*.h")
file(GLOB_RECURSE ALL_BENCHMARK_SRCS "autolink/benchmark/*.cpp" "autolink/benchmark/*.c")
file(GLOB_RECURSE ALL_EXAMPLES_HDRS "autolink/examples/*.hpp" "autolink/examples/*.h")
file(GLOB_RECURSE ALL_EXAMPLES_SRCS "autolink/examples/*.cpp" "autolink/examples/*.c")

# Select proper assembly swap implementation per architecture
set(ASM_SRCS)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|AARCH64|arm64|ARM64")
  list(APPEND ASM_SRCS ${PROJECT_SOURCE_DIR}/autolink/croutine/detail/swap_aarch64.S)
else()
  list(APPEND ASM_SRCS ${PROJECT_SOURCE_DIR}/autolink/croutine/detail/swap_x86_64.S)
endif()
file(GLOB_RECURSE TEST_LIBRARY_HDRS 
  "autolink/test/*.hpp" 
  "autolink/fake_*.hpp" 
  "autolink/*test_helpers*.hpp" 
  "autolink/*test*.hpp" 
  "autolink/mock_*.hpp")
file(GLOB_RECURSE TEST_LIBRARY_SRCS 
  "autolink/test/*.cpp" 
  "autolink/fake_*.cpp" 
  "autolink/*test_helpers*.cpp" 
  "autolink/*test*.cpp" 
  "autolink/mock_*.cpp")
file(GLOB_RECURSE ALL_TESTS "autolink/*_test.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "autolink/*_main.cpp")

# Remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
endif()

list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_AVIZ_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_AVIZ_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_BENCHMARK_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_BENCHMARK_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_EXAMPLES_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXAMPLES_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS
  ${PROJECT_SOURCE_DIR}/autolink/class_loader/test/plugin1.cpp
  ${PROJECT_SOURCE_DIR}/autolink/class_loader/test/plugin2.cpp)

# Exclude io module files on macOS (uses Linux-specific epoll API)
if(APPLE)
  list(REMOVE_ITEM ALL_LIBRARY_SRCS
    ${PROJECT_SOURCE_DIR}/autolink/io/poller.cpp
    ${PROJECT_SOURCE_DIR}/autolink/io/poll_handler.cpp
    ${PROJECT_SOURCE_DIR}/autolink/io/session.cpp)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS
    ${PROJECT_SOURCE_DIR}/autolink/io/poller.hpp
    ${PROJECT_SOURCE_DIR}/autolink/io/poll_handler.hpp
    ${PROJECT_SOURCE_DIR}/autolink/io/poll_data.hpp
    ${PROJECT_SOURCE_DIR}/autolink/io/session.hpp)
  # Also exclude io test files
  list(REMOVE_ITEM TEST_LIBRARY_SRCS
    ${PROJECT_SOURCE_DIR}/autolink/io/poller_test.cpp)
  list(REMOVE_ITEM ALL_TESTS
    ${PROJECT_SOURCE_DIR}/autolink/io/poller_test.cpp)
endif()

# Autolink library
add_library(${PROJECT_NAME} SHARED 
  ${ALL_LIBRARY_HDRS} 
  ${ALL_LIBRARY_SRCS} 
  ${PROTO_SRCS} 
  ${PROTO_HDRS}
  ${ASM_SRCS}
)

add_dependencies(${PROJECT_NAME} autolink_protos)

# causes the shared library to have a version number
set_target_properties(${PROJECT_NAME} PROPERTIES 
  VERSION ${AUTOLINK_VERSION}
  SOVERSION ${AUTOLINK_MAJOR_VERSION}
)

# Fast DDS (includes Fast CDR)
target_link_libraries(${PROJECT_NAME} PRIVATE ${FastDDS_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${FastDDS_INCLUDE_DIRS})

# Protobuf
target_link_libraries(${PROJECT_NAME} PUBLIC ${Protobuf_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${Protobuf_INCLUDE_DIRS})

# UUID
target_link_libraries(${PROJECT_NAME} PRIVATE ${UUID_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${UUID_INCLUDE_DIRS})

# Python
target_link_libraries(${PROJECT_NAME} PRIVATE Python3::Python)
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})

# Logging
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${GLOG_INCLUDE_DIRS})

# Curses (ncurses)
target_link_libraries(${PROJECT_NAME} PUBLIC ${CURSES_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CURSES_INCLUDE_DIR})

# TinyXML2
target_link_directories(${PROJECT_NAME} PUBLIC ${TinyXML2_LIBRARY_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${TinyXML2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${TinyXML2_LIBRARIES})

# Abseil (required by protobuf JSON utilities and hash functions)
# Link all absl libraries that protobuf might need
# Based on pkg-config output, protobuf requires many absl libraries
target_link_libraries(${PROJECT_NAME} PUBLIC 
    absl::status 
    absl::statusor
    absl::hash
    absl::strings
    absl::base
    absl::raw_logging_internal
)

# Fix FastCDR TEMPLATE_SPEC issue - config.h is included in autolink.hpp and transport headers

# Atomic and gflags for 128-bit atomics and SetCommandLineOption
# On macOS, atomic operations are built-in, no need to link libatomic
target_link_directories(${PROJECT_NAME} PUBLIC ${Gflags_LIBRARY_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Gflags_INCLUDE_DIRS})
if(APPLE)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${Gflags_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC atomic ${Gflags_LIBRARIES})
endif()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/autolink/proto>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/autolink>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# autolink.mainboard.launcher
google_binary(mainboard
  SRCS
    autolink/mainboard/mainboard_main.cpp
)
target_link_libraries(mainboard PUBLIC ${FastDDS_LIBRARIES})

# autolink_recorder
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/autolink/tools/autolink_recorder/autolink_recorder_main.cpp)
  google_binary(autolink_recorder
    SRCS
      autolink/tools/autolink_recorder/autolink_recorder_main.cpp
  )
  target_link_libraries(autolink_recorder PUBLIC ${PROJECT_NAME})
  target_link_libraries(autolink_recorder PUBLIC ${FastDDS_LIBRARIES})
endif()

# autolink_monitor
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/autolink/tools/autolink_monitor/autolink_monitor_main.cpp)
  google_binary(autolink_monitor
    SRCS
      autolink/tools/autolink_monitor/autolink_monitor_main.cpp
  )
  target_link_libraries(autolink_monitor PUBLIC ${PROJECT_NAME})
  target_link_libraries(autolink_monitor PUBLIC ${FastDDS_LIBRARIES})
  target_include_directories(autolink_monitor PRIVATE ${CURSES_INCLUDE_DIR})
endif()


if (${BUILD_EXAMPLES})
  add_subdirectory(autolink/examples)
endif()

## Test unit
if (${BUILD_TEST})
  set(TEST_LIB autolink_test_library)
  add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
  target_include_directories(${TEST_LIB} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY})
  target_link_libraries(${TEST_LIB} PUBLIC ${PROJECT_NAME})
  # Ensure RTPS-dependent tests link with Fast DDS
  target_link_libraries(${TEST_LIB} PUBLIC ${FastDDS_LIBRARIES})
  # Fix FastCDR TEMPLATE_SPEC issue for test library - config.h is included in headers

  add_library(autolink_class_loader_plugin1 SHARED
    autolink/class_loader/test/plugin1.cpp)
  target_link_libraries(autolink_class_loader_plugin1 PUBLIC ${PROJECT_NAME})
  target_include_directories(autolink_class_loader_plugin1 PUBLIC
    ${PROJECT_SOURCE_DIR})
  set_target_properties(autolink_class_loader_plugin1 PROPERTIES
    OUTPUT_NAME plugin1)

  add_library(autolink_class_loader_plugin2 SHARED
    autolink/class_loader/test/plugin2.cpp)
  target_link_libraries(autolink_class_loader_plugin2 PUBLIC ${PROJECT_NAME})
  target_include_directories(autolink_class_loader_plugin2 PUBLIC
    ${PROJECT_SOURCE_DIR})
  set_target_properties(autolink_class_loader_plugin2 PROPERTIES
    OUTPUT_NAME plugin2)

  add_dependencies(${TEST_LIB}
    autolink_class_loader_plugin1
    autolink_class_loader_plugin2)
  target_compile_definitions(${TEST_LIB} PRIVATE
    CLASS_LOADER_PLUGIN1_PATH="$<TARGET_FILE:autolink_class_loader_plugin1>"
    CLASS_LOADER_PLUGIN2_PATH="$<TARGET_FILE:autolink_class_loader_plugin2>")

  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    # Replace slashes as required for CMP0037.
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    if(${BUILD_GRPC})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC grpc++)
    endif()
    if(${BUILD_PROMETHEUS})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${ZLIB_LIBRARIES})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-core)
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC prometheus-cpp-pull)
    endif()
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
    # On macOS, atomic operations are built-in, no need to link libatomic
    if(NOT APPLE)
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC atomic)
    endif()
    if("${REL_FIL}" STREQUAL "autolink/class_loader/class_loader_test.cpp")
      add_dependencies("${TEST_TARGET_NAME}"
        autolink_class_loader_plugin1
        autolink_class_loader_plugin2)
      target_compile_definitions("${TEST_TARGET_NAME}" PRIVATE
        CLASS_LOADER_PLUGIN1_PATH="$<TARGET_FILE:autolink_class_loader_plugin1>"
        CLASS_LOADER_PLUGIN2_PATH="$<TARGET_FILE:autolink_class_loader_plugin2>")
    endif()
  endforeach()
endif()

# Build benchmark
if (${BUILD_BENCHMARK})
  add_subdirectory(benchmark)
endif()

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGET_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install C/C++ headers from source tree
install(DIRECTORY ${PROJECT_SOURCE_DIR}/autolink/
        DESTINATION include/autolink
        FILES_MATCHING
        PATTERN "*.h" PATTERN "*.hpp")

# Install generated protobuf headers from build tree
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/autolink/
        DESTINATION include/autolink
        FILES_MATCHING
        PATTERN "*.pb.h")

# Install generated Python protobuf files
install(DIRECTORY ${PROTO_PY_OUT_DIR}/autolink/
        DESTINATION python/autolink
        FILES_MATCHING
        PATTERN "*_pb2.py")

# Ensure __init__.py files exist for Python packages
install(SCRIPT ${PROJECT_SOURCE_DIR}/cmake/scripts/install_python_inits.cmake)

# autolink conf
install(FILES ${PROJECT_SOURCE_DIR}/autolink/conf/autolink.pb.conf 
  DESTINATION etc/${PROJECT_NAME}
  RENAME autolink.conf)
