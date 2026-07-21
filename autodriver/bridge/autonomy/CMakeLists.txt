# Copyright 2026 Autodriver contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(_AUTODRIVER_BRIDGE_SRC_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../../autodriver/bridge/autonomy)

add_library(autodriver_autonomy_bridge
  ${_AUTODRIVER_BRIDGE_SRC_DIR}/sample_converter.cpp
  ${_AUTODRIVER_BRIDGE_SRC_DIR}/collator_sink.cpp
  ${_AUTODRIVER_BRIDGE_SRC_DIR}/hub_config_loader.cpp
)

target_include_directories(autodriver_autonomy_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../autodriver>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../..>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(autodriver_autonomy_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(autodriver_autonomy_bridge
  PUBLIC
    autodriver
    ${PROJECT_NAME}
)

install(
  TARGETS autodriver_autonomy_bridge
  EXPORT autodriverTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
