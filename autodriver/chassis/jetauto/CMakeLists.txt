# Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

# JetAuto chassis: independent shared library + Autolink Component.
# Output: libautodriver_jetauto.so
#   - ChassisDriver backend "jetauto" / "hiwonder"
#   - JetAutoComponent for mainboard DAG
#
# Usage (DAG):
#   module_library : "libautodriver_jetauto.so"
#   class_name     : "autodriver::chassis::jetauto::JetAutoComponent"
#   config_file_path : "chassis/jetauto.yaml"
#
# Usage (process): link autodriver_main against autodriver_jetauto so the
# REGISTER_CHASSIS_BACKEND static registrar is pulled in.

if(NOT TARGET autodriver)
  message(FATAL_ERROR
    "chassis/jetauto requires target autodriver "
    "(add_subdirectory(autodriver) before chassis)")
endif()
if(NOT TARGET autolink)
  message(FATAL_ERROR "chassis/jetauto requires target autolink")
endif()

set(AUTODRIVER_JETAUTO_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/rrc_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/jetauto_component.cpp
)

add_library(autodriver_jetauto SHARED ${AUTODRIVER_JETAUTO_SOURCES})
add_library(autodriver::jetauto ALIAS autodriver_jetauto)

set_target_properties(autodriver_jetauto PROPERTIES
  OUTPUT_NAME "autodriver_jetauto"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

target_include_directories(autodriver_jetauto
  PUBLIC
    $<BUILD_INTERFACE:${AUTODRIVER_ROOT_DIR}>
    $<BUILD_INTERFACE:${AUTODRIVER_ROOT_DIR}/autodriver>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(autodriver_jetauto
  PUBLIC
    autodriver
  PRIVATE
    autolink
)

message(STATUS
  "autodriver: JetAuto component library → libautodriver_jetauto"
  "${CMAKE_SHARED_LIBRARY_SUFFIX}")
