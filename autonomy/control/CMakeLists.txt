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
# autonomy/control — motion controllers (TEB / MPPI / …)
#
# Builds
#   autonomy_control / autonomy::control
#   autonomy.control
#
# Toggle
#   AUTONOMY_BUILD_CONTROL
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
autocmake_find(Ipopt PRIVATE QUIET)

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_CONTROL_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_CONTROL_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_CONTROL_SRCS EXCLUDE REGEX "_test\\.cpp$")

# Vendored DAQP (C) used by sando MIQP.
file(GLOB AUTONOMY_DAQP_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/controller/sando_controller/miqp/daqp/src/*.c")
set(AUTONOMY_DAQP_INCLUDE
  "${CMAKE_CURRENT_SOURCE_DIR}/controller/sando_controller/miqp/daqp/include")
set_source_files_properties(${AUTONOMY_DAQP_SRCS} PROPERTIES
  COMPILE_DEFINITIONS "PROFILING"
  INCLUDE_DIRECTORIES "${AUTONOMY_DAQP_INCLUDE}")
list(APPEND AUTONOMY_CONTROL_SRCS ${AUTONOMY_DAQP_SRCS})

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_control
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_CONTROL_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_map
    $<TARGET_NAME_IF_EXISTS:Ipopt::Ipopt>)
add_library(autonomy::control ALIAS autonomy_control)

# Classic Ipopt package vars when no imported target exists.
if(Ipopt_FOUND AND NOT TARGET Ipopt::Ipopt)
  target_link_libraries(autonomy_control PUBLIC ${IPOPT_LIBRARIES})
endif()

target_include_directories(autonomy_control PRIVATE "${AUTONOMY_DAQP_INCLUDE}")

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.control
  NO_EXPORT
  SOURCES control_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

# --- install -----------------------------------------------------------------
autocmake_install_share(control
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
