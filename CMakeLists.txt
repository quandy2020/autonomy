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
# 1. Bootstrap (gz: gz_configure_project + module API)
# =============================================================================
set(AUTONOMY_SUPERPROJECT ON)
list(APPEND CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/cmake"
  "${PROJECT_SOURCE_DIR}/cmake/modules")
include(autonomy_module)
autonomy_configure_project()

# =============================================================================
# 2. Product options
# =============================================================================
option(BUILD_GRPC "Build autonomy gRPC support" ON)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" OFF)
option(BUILD_DOCS "Build autonomy docs support" ON)
option(BUILD_TEST "Build autonomy unit test" OFF)
option(BUILD_TOOLS "Build autonomy tools sources" ON)
option(BUILD_GRID_MAP_DEMOS "Build ROS-free grid_map demos" OFF)

option(BUILD_AUTODRIVER "Build autodriver sensor HAL subproject" ON)
option(BUILD_AUTOVIZ "Build native 3D visualization tool (autoviz)" ON)
option(BUILD_AUTOSIM "Build autonomous simulation tool (autosim)" OFF)
option(BUILD_ORBISVIEW "Build OrbisView web HMI (autonomy/orbisview)" ON)

option(BUILD_ONNXRUNTIME "Build ONNX Runtime inference support" ON)
option(BUILD_TENSORRT "Build TensorRT inference support" OFF)
option(BUILD_SHERPA_ONNX "Build sherpa-onnx ASR support for audio" OFF)

# =============================================================================
# 3. Super-project: modules, deps, embed, codegen
# =============================================================================
include(autonomy_superproject)

autonomy_superproject_bootstrap_modules()
autonomy_find_dependencies()

if(BUILD_DOCS)
  find_package(Sphinx)
  if(SPHINX_FOUND)
    add_subdirectory(docs)
  endif()
endif()

autonomy_embed_autolink()
autonomy_superproject_prepare_codegen()

# =============================================================================
# 4. Build domain libraries + umbrella target
# =============================================================================
autonomy_configure_build()

if(TARGET autolink_channel)
  target_link_libraries(autolink_channel PRIVATE autonomy)
  target_include_directories(autolink_channel PRIVATE
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR})
endif()

# =============================================================================
# 5. Companion packages (optional)
# =============================================================================
if(BUILD_AUTOVIZ)
  set(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY ON)
  add_subdirectory(autoviz)
  unset(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY)
endif()

if(BUILD_AUTOSIM)
  autonomy_embed_autosim()
endif()

if(BUILD_ORBISVIEW)
  add_subdirectory(autonomy/orbisview)
endif()

# =============================================================================
# 6. Tests + install/export
# =============================================================================
autonomy_create_packages()
