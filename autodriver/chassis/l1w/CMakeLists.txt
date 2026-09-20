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

# GENISOM 钢镚 L1-W (ZSL-1W): independent shared library + Autolink Component.
# Output: libautodriver_l1w.so
#
# Official HighLevel SDK (optional):
#   -DGenisomL1w_ROOT=/path/to/genisom_l1_sdk[_old]
#   or GENISOM_L1W_SDK_ROOT / ZSIBOT_L1_SDK
# Without SDK the library still builds; hardware Start requires simulate=true.
#
# DAG:
#   module_library : "libautodriver_l1w.so"
#   class_name     : "autodriver::chassis::l1w::L1wComponent"

if(NOT TARGET autodriver)
  message(FATAL_ERROR
    "chassis/l1w requires target autodriver "
    "(add_subdirectory(autodriver) before chassis)")
endif()
if(NOT TARGET autolink)
  message(FATAL_ERROR "chassis/l1w requires target autolink")
endif()

option(AUTODRIVER_WITH_GENISOM_L1W
  "Link official GENISOM ZSL-1W HighLevel SDK when found" ON)

set(AUTODRIVER_L1W_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/l1w_component.cpp
)

add_library(autodriver_l1w SHARED ${AUTODRIVER_L1W_SOURCES})
add_library(autodriver::l1w ALIAS autodriver_l1w)

set_target_properties(autodriver_l1w PROPERTIES
  OUTPUT_NAME "autodriver_l1w"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

target_include_directories(autodriver_l1w
  PUBLIC
    $<BUILD_INTERFACE:${AUTODRIVER_ROOT_DIR}>
    $<BUILD_INTERFACE:${AUTODRIVER_ROOT_DIR}/autodriver>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(autodriver_l1w
  PUBLIC
    autodriver
  PRIVATE
    autolink
)

set(_l1w_have_sdk FALSE)
if(AUTODRIVER_WITH_GENISOM_L1W)
  find_package(GenisomL1w QUIET)
  if(GenisomL1w_FOUND)
    target_link_libraries(autodriver_l1w PRIVATE GenisomL1w::GenisomL1w)
    target_compile_definitions(autodriver_l1w PUBLIC AUTODRIVER_HAVE_GENISOM_L1W)
    set(_l1w_have_sdk TRUE)
    message(STATUS
      "autodriver: Genisom L1-W SDK linked (${GenisomL1w_INCLUDE_DIR})")
  else()
    message(STATUS
      "autodriver: Genisom L1-W SDK not found; l1w builds simulate-only "
      "(set GenisomL1w_ROOT or GENISOM_L1W_SDK_ROOT)")
  endif()
endif()

if(_l1w_have_sdk)
  message(STATUS "autodriver: L1-W component → libautodriver_l1w (SDK linked)")
else()
  message(STATUS "autodriver: L1-W component → libautodriver_l1w (simulate-only)")
endif()
