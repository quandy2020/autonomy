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

# Robot-body HAL sources compile into libautodriver (sibling of autodriver/).
# Include as: #include "chassis/...."

if(NOT TARGET autodriver)
  message(FATAL_ERROR "chassis requires target autodriver (add_subdirectory(autodriver) first)")
endif()

target_sources(autodriver PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/chassis_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/locomotion_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/capability.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/operational_mode.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/safety_gate.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tool_command.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/stub/driver.cpp
)

# Headers live under ${AUTODRIVER_ROOT_DIR}/chassis/ → #include "chassis/..."
target_include_directories(autodriver
  PUBLIC
    $<BUILD_INTERFACE:${AUTODRIVER_ROOT_DIR}>
)
