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

# Build Base as a loadable autolink component only when an inference backend
# and the component runtime are available. Registration stays on this DSO.
set(_base_backend FALSE)
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  set(_base_backend TRUE)
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  set(_base_backend TRUE)
endif()
if(NOT _base_backend)
  return()
endif()
unset(_base_backend)

include(autonomy_module)
autonomy_component(base_component
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
    tasks/depth/moge.cpp)

if(BUILD_TEST AND TARGET base_component)
  set(_base_component_test autonomy.perception.base.component_test)
  autonomy_test(${_base_component_test} base_component_test.cpp base_component)
  add_dependencies(${_base_component_test} base_component)
  unset(_base_component_test)
endif()
