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
#   autonomy.recorder          (AUTONOMY_BUILD_RECORDER)
#
# Toggle
#   AUTONOMY_BUILD_SYSTEM
#   AUTONOMY_BUILD_RECORDER
#   BUILD_PROMETHEUS
# -----------------------------------------------------------------------------

option(AUTONOMY_BUILD_RECORDER
  "Build autonomy.recorder bypass evidence process" ON)

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_SYSTEM_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_SYSTEM_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_SYSTEM_SRCS EXCLUDE REGEX "_test\\.cpp$")
if(NOT AUTONOMY_BUILD_RECORDER)
  list(FILTER AUTONOMY_SYSTEM_SRCS EXCLUDE REGEX "/recorder/")
endif()

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
    autonomy::cli11
)

autocmake_binary(autonomy.ota
  NO_EXPORT
  SOURCES ota/ota_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

if(AUTONOMY_BUILD_RECORDER)
  autocmake_binary(autonomy.recorder
    NO_EXPORT
    SOURCES recorder/recorder_main.cpp
    DEPENDENCIES
      autonomy
      glog::glog
      autonomy::cli11
  )
  if(BUILD_TEST AND NOT BUILD_TESTING)
    set(BUILD_TESTING ON)
  endif()
  autocmake_test(system.recorder_test
    SOURCES recorder/recorder_test.cpp
    DEPENDENCIES autonomy_system)
endif()

# --- install -----------------------------------------------------------------
autocmake_install_share(system
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch"
  SCRIPTS_PATTERN "*.sh")
