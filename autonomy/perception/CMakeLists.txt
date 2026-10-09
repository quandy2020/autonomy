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
# autonomy/perception — perception stack
#
# Builds
#   autonomy_perception / autonomy::autonomy_perception
#   autonomy.perception
#   base/ follow/   optional subdirs (base self-guards on infer SDKs)
#
# Toggle
#   AUTONOMY_BUILD_PERCEPTION
#   BUILD_ONNXRUNTIME / BUILD_TENSORRT  (from common)
# -----------------------------------------------------------------------------

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_PERCEPTION_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_PERCEPTION_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_PERCEPTION_SRCS EXCLUDE REGEX "_test\\.cpp$")
list(FILTER AUTONOMY_PERCEPTION_SRCS EXCLUDE REGEX "/(base|follow)/")

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_perception
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_PERCEPTION_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_map
    $<TARGET_NAME_IF_EXISTS:autonomy_opencv>)
add_library(autonomy::perception ALIAS autonomy_perception)

# Optional ML backends (discovered by autonomy/common).
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  target_compile_definitions(autonomy_perception PUBLIC AUTONOMY_HAS_ONNXRUNTIME)
  target_include_directories(autonomy_perception SYSTEM PUBLIC ${OnnxRuntime_INCLUDE_DIRS})
  target_link_libraries(autonomy_perception PUBLIC ${OnnxRuntime_LIBRARIES})
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  target_compile_definitions(autonomy_perception PUBLIC AUTONOMY_HAS_TENSORRT)
  target_include_directories(autonomy_perception SYSTEM PUBLIC ${TensorRT_INCLUDE_DIRS})
  target_link_libraries(autonomy_perception PUBLIC
    ${TensorRT_LIBRARIES}
    $<TARGET_NAME_IF_EXISTS:CUDA::cudart>)
endif()

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.perception
  NO_EXPORT
  SOURCES perception_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    gflags::gflags)

add_subdirectory(base)
add_subdirectory(follow)

# --- install -----------------------------------------------------------------
autocmake_install_share(perception
  CONF_PATTERN "*.pb.txt"
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
