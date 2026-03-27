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

cmake_minimum_required(VERSION 3.14)

project(autoviz LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AUTOVIZ_BUILD_TEST "Build unit tests" OFF)
option(AUTOVIZ_BUILD_EXAMPLE "Build example" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
# Prefer local find modules under autoviz/cmake.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules")

find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Mcap QUIET)
find_package(nlohmann_json QUIET)
find_package(Protobuf REQUIRED)

set(AUTOVIZ_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(AUTOVIZ_PROTO_GEN "${CMAKE_CURRENT_BINARY_DIR}/autoviz_proto")
file(MAKE_DIRECTORY ${AUTOVIZ_PROTO_GEN})
set(AUTOVIZ_CONF_PROTO "${AUTOVIZ_PROTO_DIR}/autoviz_conf.proto")
set(AUTOVIZ_PROTO_SRC "${AUTOVIZ_PROTO_GEN}/autoviz_conf.pb.cc")
set(AUTOVIZ_PROTO_HDR "${AUTOVIZ_PROTO_GEN}/autoviz_conf.pb.h")
add_custom_command(
  OUTPUT ${AUTOVIZ_PROTO_SRC} ${AUTOVIZ_PROTO_HDR}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS --cpp_out=${AUTOVIZ_PROTO_GEN}
       -I${AUTOVIZ_PROTO_DIR}
       ${AUTOVIZ_CONF_PROTO}
  DEPENDS ${AUTOVIZ_CONF_PROTO}
  COMMENT "Generate autoviz_conf.pb.cc"
  VERBATIM)

# Reuse in-workspace targets when building as part of autonomy.
# Fallback to installed packages for standalone builds.
set(AUTOVIZ_AUTOLINK_TARGET "")
if(TARGET autolink)
  set(AUTOVIZ_AUTOLINK_TARGET autolink)
elseif(TARGET autolink::autolink)
  set(AUTOVIZ_AUTOLINK_TARGET autolink::autolink)
else()
  find_package(autolink REQUIRED)
  if(TARGET autolink)
    set(AUTOVIZ_AUTOLINK_TARGET autolink)
  elseif(TARGET autolink::autolink)
    set(AUTOVIZ_AUTOLINK_TARGET autolink::autolink)
  else()
    message(FATAL_ERROR "autolink target not found after find_package(autolink).")
  endif()
endif()

set(AUTOVIZ_AUTOMSGS_TARGET "")
if(TARGET automsgs)
  set(AUTOVIZ_AUTOMSGS_TARGET automsgs)
elseif(TARGET automsgs::automsgs)
  set(AUTOVIZ_AUTOMSGS_TARGET automsgs::automsgs)
else()
  find_package(automsgs REQUIRED)
  if(TARGET automsgs)
    set(AUTOVIZ_AUTOMSGS_TARGET automsgs)
  elseif(TARGET automsgs::automsgs)
    set(AUTOVIZ_AUTOMSGS_TARGET automsgs::automsgs)
  else()
    message(FATAL_ERROR "automsgs target not found after find_package(automsgs).")
  endif()
endif()

# -----------------------------------------------------------------------------
# autoviz core library and executable (autolink + automsgs only)
# -----------------------------------------------------------------------------
add_library(autoviz_core STATIC
  core/app.cpp
  ${AUTOVIZ_PROTO_SRC}
  core/recorder/recorder.cpp
  core/foxglove/foxglove_server.cpp
  core/autolink/autolink_bridge.cpp
  core/convert/schema/protobuf_ros_style_name.cpp
  core/convert/schema/foxglove_channel_schema.cpp
  core/convert/registry/automsgs_foxglove_registry.cpp
  core/convert/message/convert_nav_path_to_scene.cpp
  core/convert/message/convert_automsgs_body_3d.cpp
  core/convert/message/convert_automsgs_message.cpp
)
target_include_directories(autoviz_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../autolink
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  ${AUTOVIZ_PROTO_GEN}
)
target_link_libraries(autoviz_core PUBLIC
  ${AUTOVIZ_AUTOLINK_TARGET}
  ${AUTOVIZ_AUTOMSGS_TARGET}
  protobuf::libprotobuf
)

# -----------------------------------------------------------------------------
# Foxglove SDK (required for websocket streaming)
# -----------------------------------------------------------------------------
# When built inside this monorepo, foxglove is vendored under the top-level build
# directory as `_deps/foxglove-src`. We link directly to the produced `libfoxglove.so`
# so `autoviz` can run independently of other visualization targets.
set(FOXGLOVE_DEPS_INCLUDE_DIR "${CMAKE_BINARY_DIR}/_deps/foxglove-src/include")
set(FOXGLOVE_DEPS_LIB_A_PATH "${CMAKE_BINARY_DIR}/_deps/foxglove-src/lib/libfoxglove.a")
set(FOXGLOVE_DEPS_LIB_SO_PATH "${CMAKE_BINARY_DIR}/_deps/foxglove-src/lib/libfoxglove.so")

if (EXISTS "${FOXGLOVE_DEPS_INCLUDE_DIR}")
  target_include_directories(autoviz_core PUBLIC "${FOXGLOVE_DEPS_INCLUDE_DIR}")
  # The foxglove SDK provides C++ wrappers in `libfoxglove.a` and some runtime
  # symbols in `libfoxglove.so`, so link both to satisfy all dependencies.
  target_link_libraries(autoviz_core PUBLIC "${FOXGLOVE_DEPS_LIB_A_PATH}" "${FOXGLOVE_DEPS_LIB_SO_PATH}")
  target_compile_definitions(autoviz_core PUBLIC AUTOVIZ_HAS_FOXGLOVE=1)

  # 预编译包（如 sdk v0.11.0 zip）的 C++ 头可能不含各 schema 的 encode()/static schema()；
  # 与仓库内完整 foxglove-sdk 区分，避免编译失败。
  set(AUTOVIZ_FOXGLOVE_HAS_ENCODE 0)
  if(EXISTS "${FOXGLOVE_DEPS_INCLUDE_DIR}/foxglove/schemas.hpp")
    file(READ "${FOXGLOVE_DEPS_INCLUDE_DIR}/foxglove/schemas.hpp" _autoviz_foxglove_schemas_hpp)
    string(FIND "${_autoviz_foxglove_schemas_hpp}" "FoxgloveError encode" _autoviz_foxglove_encode_pos)
    string(FIND "${_autoviz_foxglove_schemas_hpp}" "schema()" _autoviz_foxglove_schema_pos)
    if(NOT _autoviz_foxglove_encode_pos EQUAL -1 AND NOT _autoviz_foxglove_schema_pos EQUAL -1)
      set(AUTOVIZ_FOXGLOVE_HAS_ENCODE 1)
    endif()
  endif()
  if(AUTOVIZ_FOXGLOVE_HAS_ENCODE)
    target_compile_definitions(autoviz_core PUBLIC AUTOVIZ_FOXGLOVE_HAS_ENCODE=1)
    message(STATUS "autoviz: Foxglove C++ encode()/schema() detected — full automsgs→Foxglove conversion enabled.")
  else()
    message(STATUS "autoviz: Foxglove without per-schema encode()/schema() — schema_registry only; optional converters stubbed.")
  endif()
else()
  message(WARNING "foxglove headers/libs not found under ${CMAKE_BINARY_DIR}/_deps/foxglove-src; autoviz websocket server may not build.")
endif()

# When built inside this monorepo, `libautonomy.so` already contains the C++
# foxglove wrapper implementations. Link it so `autoviz` doesn't need to
# compile foxglove wrapper sources again.
set(AUTONOMY_CORE_LIB_PATH "${CMAKE_BINARY_DIR}/lib/libautonomy.so")
if (EXISTS "${AUTONOMY_CORE_LIB_PATH}")
  target_link_libraries(autoviz_core PUBLIC "${AUTONOMY_CORE_LIB_PATH}")
endif()

if(DEFINED AUTOMSGS_INCLUDE_DIRS)
  target_include_directories(autoviz_core PUBLIC ${AUTOMSGS_INCLUDE_DIRS})
endif()
if(MCAP_FOUND)
  # mcap writer.hpp only includes writer.inl when MCAP_IMPLEMENTATION is
  # defined (implementation is not fully header-only in this configuration).
  target_compile_definitions(autoviz_core PUBLIC AUTOVIZ_HAS_MCAP=1 MCAP_IMPLEMENTATION)
  target_include_directories(autoviz_core PUBLIC ${MCAP_INCLUDE_DIRS})
  if(MCAP_LIBRARIES)
    target_link_libraries(autoviz_core PUBLIC ${MCAP_LIBRARIES})
  endif()
endif()

add_executable(autoviz core/main.cpp)
target_link_libraries(autoviz PRIVATE autoviz_core)

# -----------------------------------------------------------------------------
# Example (optional)
# -----------------------------------------------------------------------------
if(AUTOVIZ_BUILD_EXAMPLE)
  add_subdirectory(example)
endif()

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS autoviz RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config/autoviz.pb.conf"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/autoviz)
if(AUTOVIZ_BUILD_EXAMPLE)
  install(TARGETS autoviz.example.datas_publisher RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# -----------------------------------------------------------------------------
# Tests (optional): registry 查找（不依赖 Foxglove encode）
# -----------------------------------------------------------------------------
if(AUTOVIZ_BUILD_TEST)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    enable_testing()
    add_executable(autoviz_registry_test
      core/convert/registry/automsgs_foxglove_registry_test.cpp)
    target_link_libraries(autoviz_registry_test PRIVATE
      autoviz_core
      GTest::gtest_main)
    add_test(NAME autoviz_registry_test COMMAND autoviz_registry_test)
    message(STATUS "autoviz: autoviz_registry_test enabled (GTest)")
  else()
    message(STATUS "autoviz: GTest not found — set AUTOVIZ_BUILD_TEST but no tests built")
  endif()
endif()

# Regenerate automsgs ↔ Foxglove registry when protos change (optional dev target).
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_custom_target(autoviz_regenerate_foxglove_registry
    COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/core/convert/tools/generate_automsgs_foxglove_registry.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Regenerate convert/registry/automsgs_foxglove_registry_data.hpp"
    VERBATIM)
endif()
