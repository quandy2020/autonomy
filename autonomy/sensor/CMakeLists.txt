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
# autonomy/sensor — sensor drivers / adapters
#
# Builds
#   autonomy_sensor / autonomy::autonomy_sensor
#
# Toggle
#   AUTONOMY_BUILD_SENSOR
# -----------------------------------------------------------------------------

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_SENSOR_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_SENSOR_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_SENSOR_SRCS EXCLUDE REGEX "_test\\.cpp$")

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_sensor
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_SENSOR_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_control)
add_library(autonomy::sensor ALIAS autonomy_sensor)