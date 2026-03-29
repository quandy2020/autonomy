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

# -----------------------------------------------------------------------------
# proto（参考 autonomy/autolink/CMakeLists.txt：单次 protoc、custom_target、core 依赖）
# -----------------------------------------------------------------------------
set(AUTOLINK_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(AUTOLINK_PROTO_GEN_DIR "${CMAKE_BINARY_DIR}/autolink/proto")
file(MAKE_DIRECTORY "${AUTOLINK_PROTO_GEN_DIR}")

file(GLOB AUTOLINK_PROTO_FILES "${AUTOLINK_PROTO_DIR}/*.proto")

set(AUTOLINK_PROTO_SRCS)
set(AUTOLINK_PROTO_HDRS)
set(AUTOLINK_PROTO_INPUTS)

foreach(PROTO_FILE ${AUTOLINK_PROTO_FILES})
  get_filename_component(PROTO_NAME "${PROTO_FILE}" NAME_WE)
  get_filename_component(PROTO_BASENAME "${PROTO_FILE}" NAME)
  list(APPEND AUTOLINK_PROTO_SRCS "${AUTOLINK_PROTO_GEN_DIR}/${PROTO_NAME}.pb.cc")
  list(APPEND AUTOLINK_PROTO_HDRS "${AUTOLINK_PROTO_GEN_DIR}/${PROTO_NAME}.pb.h")
  list(APPEND AUTOLINK_PROTO_INPUTS "${PROTO_BASENAME}")
endforeach()

add_custom_command(
  OUTPUT ${AUTOLINK_PROTO_SRCS} ${AUTOLINK_PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    --proto_path=${AUTOLINK_PROTO_DIR}
    --cpp_out=${AUTOLINK_PROTO_GEN_DIR}
    ${AUTOLINK_PROTO_INPUTS}
  WORKING_DIRECTORY ${AUTOLINK_PROTO_DIR}
  DEPENDS ${AUTOLINK_PROTO_FILES}
  COMMENT "Generating C++ protobuf files"
)

add_custom_target(autolink_protos DEPENDS ${AUTOLINK_PROTO_SRCS} ${AUTOLINK_PROTO_HDRS})

# -----------------------------------------------------------------------------
# core 库与 mainboard
# -----------------------------------------------------------------------------
file(GLOB_RECURSE AUTOLINK_CORE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/action/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/base/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/blocker/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/class_loader/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/component/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/context/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/croutine/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/data/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/event/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/io/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/logger/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/message/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/node/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/parameter/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/plugin_manager/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/profiler/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/record/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/scheduler/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/service/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/service_discovery/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/sysmo/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/task/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/time/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/timer/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/binary.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/autolink.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/cyber.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/init.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/state.cpp"
)

# 排除单元测试、示例和工具等非核心源码
list(FILTER AUTOLINK_CORE_SOURCES EXCLUDE REGEX ".*_test.cpp$")
list(FILTER AUTOLINK_CORE_SOURCES EXCLUDE REGEX ".*/tools/.*")
list(FILTER AUTOLINK_CORE_SOURCES EXCLUDE REGEX ".*/mainboard/.*")
list(FILTER AUTOLINK_CORE_SOURCES EXCLUDE REGEX ".*/example/.*")
list(FILTER AUTOLINK_CORE_SOURCES EXCLUDE REGEX ".*/integration_test/.*")

add_library(autolink SHARED ${AUTOLINK_CORE_SOURCES} ${AUTOLINK_PROTO_SRCS})
add_dependencies(autolink autolink_protos)

# croutine 协程上下文切换汇编（按架构二选一，需启用 ASM 语言）
enable_language(ASM)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_sources(autolink PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/croutine/detail/swap_aarch64.S")
else()
    target_sources(autolink PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/croutine/detail/swap_x86_64.S")
endif()

target_include_directories(autolink PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    ${Protobuf_INCLUDE_DIRS}
    ${FastDDS_INCLUDE_DIRS}
)
if(abseil_FOUND)
  target_link_libraries(autolink PUBLIC abseil::absl_base)
elseif(EXISTS "/opt/homebrew/include/absl")
  target_include_directories(autolink PUBLIC "/opt/homebrew/include")
endif()

target_link_libraries(autolink PUBLIC
    ${Protobuf_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${FastDDS_LIBRARIES}
    ${RT_LIB}
    ${TinyXML2_LIB}
)

# Python embedding and tcmalloc can conflict at runtime (invalid free/abort).
# When building Python bindings, strip tcmalloc from autolink link line by default,
# but keep profiler library if available.
set(_autolink_gperftools_link_libs ${GPERFTOOLS_LIBRARIES})
if(AUTOLINK_BUILD_PYTHON AND NOT AUTOLINK_FORCE_TCMALLOC_WITH_PYTHON)
  if(_autolink_gperftools_link_libs)
    list(FILTER _autolink_gperftools_link_libs EXCLUDE REGEX ".*tcmalloc.*")
  endif()
  message(STATUS "autolink: AUTOLINK_BUILD_PYTHON=ON, linking gperftools without tcmalloc")
endif()
if(_autolink_gperftools_link_libs)
  target_link_libraries(autolink PUBLIC ${_autolink_gperftools_link_libs})
endif()
if(GFLAGS_LIB)
  target_link_libraries(autolink PUBLIC ${GFLAGS_LIB})
endif()
if(UUID_LIB)
  target_link_libraries(autolink PUBLIC ${UUID_LIB})
endif()

# 128-bit 原子操作 (__atomic_load_16 等) 在 Linux 上由 libatomic 提供
if(UNIX AND NOT APPLE)
  target_link_libraries(autolink PUBLIC atomic)
endif()

# 可执行程序：mainboard（始终构建）
add_executable(autolink_mainboard
    mainboard/mainboard.cpp
    mainboard/module_argument.cpp
    mainboard/module_controller.cpp
)
target_link_libraries(autolink_mainboard PRIVATE autolink)
# mainboard uses ProfilerStart/Stop & HeapProfiler* symbols directly.
# Keep full gperftools linkage here even when autolink core strips tcmalloc
# for Python embedding safety.
if(GPERFTOOLS_LIBRARIES)
  target_link_libraries(autolink_mainboard PRIVATE ${GPERFTOOLS_LIBRARIES})
endif()

# 可选工具：与 Autolink tools 一致，每工具一目录、每目录一 CMakeLists（构建或仅安装）
if(AUTOLINK_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

# Python bindings
if(AUTOLINK_BUILD_PYTHON)
  add_subdirectory(python)
endif()

# 单元测试
if(AUTOLINK_BUILD_TEST AND GMock_FOUND)
  # Keep test include paths aligned with source include style
  # like "autolink/common/init.hpp" used by python test headers.
  set(AUTOLINK_TEST_INCLUDE_ROOT "${AUTOLINK_ROOT_DIR}")
  if(NOT AUTOLINK_TEST_INCLUDE_ROOT)
    get_filename_component(AUTOLINK_TEST_INCLUDE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
  endif()

  file(GLOB_RECURSE ALL_TESTS "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp")
  list(FILTER ALL_TESTS EXCLUDE REGEX ".*/integration_test/.*")

  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    if(DIR)
      set(TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    else()
      set(TEST_TARGET_NAME "${FIL_WE}")
    endif()
    string(REPLACE "/" "." TEST_TARGET_NAME "${TEST_TARGET_NAME}")
    set(TEST_TARGET_NAME "autolink.${TEST_TARGET_NAME}")

    google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    set_target_properties(${TEST_TARGET_NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    target_include_directories(${TEST_TARGET_NAME} PRIVATE
      ${AUTOLINK_TEST_INCLUDE_ROOT}
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_BINARY_DIR}
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(${TEST_TARGET_NAME} PRIVATE autolink ${GMOCK_LIBRARIES})
    if(UNIX AND NOT APPLE)
      target_link_libraries(${TEST_TARGET_NAME} PRIVATE atomic)
    endif()

    # Some vendored/system headers trigger -Wmaybe-uninitialized in tests (treated as error in this build).
    # Keep production library strict; relax only for test targets.
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(${TEST_TARGET_NAME} PRIVATE -Wno-error=maybe-uninitialized)
    endif()
  endforeach()
endif()