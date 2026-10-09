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
# autonomy/perception/base — ONNX / TensorRT inference component
#
# Builds
#   base_component / autonomy::base_component
#
# Guard
#   Requires BUILD_ONNXRUNTIME or BUILD_TENSORRT with a found SDK.
# -----------------------------------------------------------------------------

# --- guard -------------------------------------------------------------------
if(NOT ((BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND) OR (BUILD_TENSORRT AND TensorRT_FOUND)))
  return()
endif()

# --- library -----------------------------------------------------------------
autocmake_library(base_component
  SHARED
  NO_EXPORT
  SOURCES
    base_component.cpp
    engine/model.cpp
    tasks/detect/detect.cpp
    tasks/segment/segment.cpp
    tasks/classify/classify.cpp
    tasks/pose/pose.cpp
    tasks/obb/obb.cpp
    tasks/track/track.cpp
    tasks/depth/depth.cpp
    tasks/depth/moge.cpp
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  PRIVATE_DEPENDENCIES
    autonomy
    autolink
    automsgs)
set_target_properties(base_component PROPERTIES OUTPUT_NAME "base_component")

if(BUILD_TEST OR BUILD_TESTING)
  autocmake_test(autonomy.perception.base.component_test
    SOURCES base_component_test.cpp
    DEPENDENCIES base_component)
endif()
