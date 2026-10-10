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
# autonomy/audio — audio / ASR components
#
# Builds
#   autonomy_audio / autonomy::audio
#   audio_component / autonomy::audio_component
#
# Toggle
#   AUTONOMY_BUILD_AUDIO
#   BUILD_SHERPA_ONNX
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
option(BUILD_SHERPA_ONNX "sherpa-onnx ASR" OFF)
autocmake_find(SherpaOnnx PRIVATE QUIET)

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_audio
  INTERFACE
  NO_EXPORT
  DEPENDENCIES autonomy_common)
add_library(autonomy::audio ALIAS autonomy_audio)

file(GLOB_RECURSE AUTONOMY_AUDIO_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_AUDIO_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_AUDIO_SRCS EXCLUDE REGEX "_test\\.cpp$")

autocmake_library(audio_component
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_AUDIO_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  PRIVATE_DEPENDENCIES
    autonomy
    autolink
    automsgs
    autonomy::yaml)
set_target_properties(audio_component PROPERTIES OUTPUT_NAME "audio_component")

if(BUILD_SHERPA_ONNX AND SherpaOnnx_FOUND)
  target_compile_definitions(audio_component PRIVATE AUTONOMY_HAS_SHERPA_ONNX)
  target_include_directories(audio_component PRIVATE ${SherpaOnnx_INCLUDE_DIRS})
  target_link_libraries(audio_component PRIVATE ${SherpaOnnx_LIBRARIES})
  message(STATUS "audio_component: sherpa-onnx ASR enabled")
else()
  message(STATUS "audio_component: ASR stub (no sherpa-onnx)")
endif()

# --- install -----------------------------------------------------------------
autocmake_install_share(audio
  CONF_PATTERN "*.pb.txt" "*.yaml"
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
