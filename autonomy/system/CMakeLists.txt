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
# autonomy/system — AutonomyOptions loader + system monitor / OTA
#
# Builds
#   autonomy_system / autonomy::autonomy_system
#   autonomy.monitor
#   autonomy.ota
#
# Toggle
#   AUTONOMY_BUILD_SYSTEM
#   BUILD_PROMETHEUS
# -----------------------------------------------------------------------------

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_SYSTEM_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_SYSTEM_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_SYSTEM_SRCS EXCLUDE REGEX "_test\\.cpp$")

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_system
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_SYSTEM_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_task)
add_library(autonomy::system ALIAS autonomy_system)

if(BUILD_PROMETHEUS)
  autocmake_find(prometheus-cpp REQUIRED PRIVATE CONFIG)
  if(NOT prometheus-cpp_FOUND)
    message(FATAL_ERROR "system: prometheus-cpp required when BUILD_PROMETHEUS=ON")
  endif()
  target_compile_definitions(autonomy_system PRIVATE USE_PROMETHEUS=1)
  target_link_libraries(autonomy_system PRIVATE
    prometheus-cpp-core prometheus-cpp-pull)
endif()

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.monitor
  NO_EXPORT
  SOURCES monitor/monitor_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    gflags::gflags)

autocmake_binary(autonomy.ota
  NO_EXPORT
  SOURCES ota/ota_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    gflags::gflags)

# --- install -----------------------------------------------------------------
autocmake_install_share(system
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch"
  SCRIPTS_PATTERN "*.sh")
