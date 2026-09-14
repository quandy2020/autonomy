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

# autonomy/audio — audio / ASR components

include(autonomy_module)
autonomy_module(audio)

# Placeholder INTERFACE library (sources live on the component DSO).
autonomy_library(INTERFACE DEPENDENCIES autonomy_common)

# Loadable component: GLOB *.cpp under this dir; drop *main.cpp / *_test.cpp.
autonomy_glob_srcs(_AUDIO_COMPONENT_SRCS)
autonomy_component(audio_component
  SOURCES ${_AUDIO_COMPONENT_SRCS}
  PRIVATE_LINK_LIBS yaml-cpp)
unset(_AUDIO_COMPONENT_SRCS)

# Optional sherpa-onnx ASR; stub engine when unavailable.
if(TARGET audio_component)
  if(BUILD_SHERPA_ONNX AND SherpaOnnx_FOUND)
    target_compile_definitions(audio_component PRIVATE AUTONOMY_HAS_SHERPA_ONNX)
    target_include_directories(audio_component PRIVATE ${SherpaOnnx_INCLUDE_DIRS})
    target_link_libraries(audio_component PRIVATE ${SherpaOnnx_LIBRARIES})
    message(STATUS "audio_component: sherpa-onnx ASR enabled")
  else()
    message(STATUS "audio_component: ASR stub (no sherpa-onnx)")
  endif()
endif()

# Runtime assets → share/autonomy/audio/ (launch / conf / dag)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/audio/conf
  FILES_MATCHING PATTERN "*.pb.txt" PATTERN "*.yaml")
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/audio/dag
  FILES_MATCHING PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/audio/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
