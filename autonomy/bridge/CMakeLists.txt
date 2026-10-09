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
# autonomy/bridge — onboard gRPC external API
#
# Builds
#   autonomy_bridge / autonomy::autonomy_bridge   shared library
#   autonomy.bridge                               process entry (bridge_main.cpp)
#
# Guard
#   BUILD_GRPC=OFF  →  this file returns early (domain stays out of the graph)
#
# Toggle
#   AUTONOMY_BUILD_BRIDGE  (from root autonomy_modules())
# -----------------------------------------------------------------------------

# --- guard -------------------------------------------------------------------
if(NOT BUILD_GRPC)
  message(STATUS "bridge: BUILD_GRPC=OFF — skipped")
  return()
endif()

# --- dependencies ------------------------------------------------------------
# Domain finds run after root autocmake_build(). autocmake_find(REQUIRED)
# only records errors for a later build pass, so FATAL here if still missing.
autocmake_find(gRPC REQUIRED PRIVATE)
if(NOT gRPC_FOUND)
  message(FATAL_ERROR "bridge: gRPC is required when BUILD_GRPC=ON")
endif()

# Optional tracing; absence keeps a no-op provider in tools/otel/.
autocmake_find(opentelemetry-cpp PRIVATE QUIET CONFIG)

# --- sources -----------------------------------------------------------------
# Library sources: everything under this dir except mains and unit tests.
file(GLOB_RECURSE AUTONOMY_BRIDGE_SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_BRIDGE_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_BRIDGE_SRCS EXCLUDE REGEX "_test\\.cpp$")

# --- library -----------------------------------------------------------------
# NO_EXPORT: installed with the autonomy umbrella, not a separate export set.
autocmake_library(autonomy_bridge
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_BRIDGE_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_system
    grpc++
    grpc)
add_library(autonomy::bridge ALIAS autonomy_bridge)

# Optional extras when the imported target exists.
target_link_libraries(autonomy_bridge PUBLIC
  $<TARGET_NAME_IF_EXISTS:gRPC::grpc++_reflection>
  $<TARGET_NAME_IF_EXISTS:opentelemetry-cpp::opentelemetry_trace>)
target_compile_definitions(autonomy_bridge PUBLIC
  $<$<TARGET_EXISTS:gRPC::grpc++_reflection>:AUTONOMY_HAVE_GRPC_REFLECTION=1>
  $<$<TARGET_EXISTS:opentelemetry-cpp::opentelemetry_trace>:AUTONOMY_HAVE_OTEL=1>)

# Vendored CLI11 (bridge_main / options). GNU-only workaround for node_client.
target_include_directories(autonomy_bridge PRIVATE
  ${PROJECT_SOURCE_DIR}/autolink/thirdparty/CLI11/include)
set_source_files_properties(node_client.cpp PROPERTIES
  COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:GNU>:-fno-ree>")

# --- binary ------------------------------------------------------------------
# Links the INTERFACE umbrella (pulls autonomy_bridge) plus logging flags.
autocmake_binary(autonomy.bridge
  NO_EXPORT
  SOURCES bridge_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    gflags::gflags)

# --- install -----------------------------------------------------------------
# Runtime assets → share/autonomy/bridge/{conf,dag,launch}.
autocmake_install_share(bridge
  CONF_PATTERN "*.pb.txt"
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
