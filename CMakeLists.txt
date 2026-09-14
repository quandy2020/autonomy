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

cmake_minimum_required(VERSION 3.20)

if(POLICY CMP0177)
  cmake_policy(SET CMP0177 NEW)
endif()

project(autonomy)

# =============================================================================
# 1. Configure project  (gz: gz_configure_project)
# =============================================================================
list(APPEND CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/cmake"
  "${PROJECT_SOURCE_DIR}/cmake/modules")
include(autonomy_module)
autonomy_configure_project()

# =============================================================================
# 2. Options  (declared in root, like gz packages)
# =============================================================================
# Core product toggles
option(BUILD_GRPC "Build autonomy gRPC support" ON)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" OFF)
option(BUILD_DOCS "Build autonomy docs support" ON)
option(BUILD_TEST "Build autonomy unit test" ON)
option(BUILD_TOOLS "Build autonomy tools sources" ON)
option(BUILD_GRID_MAP_DEMOS "Build ROS-free grid_map demos" OFF)

# Nested / companion packages
option(BUILD_AUTODRIVER "Build autodriver sensor HAL subproject" ON)
option(BUILD_AUTOVIZ "Build native 3D visualization tool (autoviz)" ON)
option(BUILD_AUTOSIM "Build autonomous simulation tool (autosim)" OFF)
option(BUILD_ORBISVIEW "Build OrbisView web HMI (autonomy/orbisview)" OFF)

# Inference / ASR
option(BUILD_ONNXRUNTIME "Build ONNX Runtime inference support" ON)
option(BUILD_TENSORRT "Build TensorRT inference support" OFF)
option(BUILD_SHERPA_ONNX "Build sherpa-onnx ASR support for audio" OFF)

set(AUTONOMY_MODULE_ORDER
  common transform map vehicle prediction control planning perception
  localization sensor task system audio bridge visualization)
include(autonomy_options)
autonomy_declare_module_options()
autonomy_validate_module_graph()
autonomy_compute_enabled_modules(AUTONOMY_ENABLED_MODULES)
foreach(_module IN LISTS AUTONOMY_MODULE_ORDER)
  string(TOUPPER "${_module}" _module_upper)
  if(NOT AUTONOMY_BUILD_${_module_upper})
    message(STATUS
      "autonomy: module '${_module}' disabled by "
      "AUTONOMY_BUILD_${_module_upper}=OFF")
  endif()
endforeach()
unset(_module)
unset(_module_upper)

# =============================================================================
# 3. Find dependencies  (gz: gz_find_package(...); stay in root)
# =============================================================================
include(autonomy_find_dependencies)
autonomy_find_dependencies()

# =============================================================================
# 4. Embedded / companion subprojects
# =============================================================================
if(BUILD_DOCS)
  find_package(Sphinx)
  if(SPHINX_FOUND)
    add_subdirectory(docs)
  endif()
endif()

include(autonomy_autolink_embed)

# =============================================================================
# 5. Modules, messages, codegen
# =============================================================================
set(AUTONOMY_MODULE_CONDITION_bridge BUILD_GRPC)
set(AUTONOMY_MODULE_CONDITION_visualization foxglove-sdk_FOUND)

autonomy_configure_tests()

add_subdirectory(automsgs)

if(BUILD_AUTODRIVER)
  include(autonomy_autodriver_embed)
endif()

include(autonomy_protobuf)

configure_file(
  "${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp.cmake"
  "${PROJECT_BINARY_DIR}/autonomy/common/config.hpp"
  @ONLY)
autonomy_configure_version()

# =============================================================================
# 6. Configure build  (gz: gz_configure_build)
# =============================================================================
autonomy_configure_build()

if(TARGET autolink_channel)
  target_link_libraries(autolink_channel PRIVATE autonomy)
  target_include_directories(autolink_channel PRIVATE
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR})
endif()

# =============================================================================
# 7. Extra packages (autoviz / autosim / orbisview)
# =============================================================================
if(BUILD_AUTOVIZ)
  set(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY ON)
  add_subdirectory(autoviz)
  unset(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY)
endif()

if(BUILD_AUTOSIM)
  include(autonomy_autosim_embed)
endif()

if(BUILD_ORBISVIEW)
  if(TARGET autolink)
    set(ORBISVIEW_WITH_AUTOLINK ON CACHE BOOL
      "Link OrbisView against Autolink" FORCE)
  endif()
  add_subdirectory(autonomy/orbisview)
endif()

# =============================================================================
# 8. Create packages  (gz: gz_create_packages)
# =============================================================================
autonomy_create_packages()
