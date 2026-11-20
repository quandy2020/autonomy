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
#   base_component / follow_component plugin targets
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
list(FILTER AUTONOMY_PERCEPTION_SRCS EXCLUDE REGEX "/(engine|inference|models|tasks|behaviors)/")

# Compiled once by autonomy_common in the common domain.
list(FILTER AUTONOMY_PERCEPTION_SRCS EXCLUDE REGEX "/data/(map_reader|sensor_observation|observation_buffer)\\.cpp$")
list(FILTER AUTONOMY_PERCEPTION_SRCS EXCLUDE REGEX "/observation/(lidar_observation_adapter|rgbd_observation_adapter|sensor_message_decoder|observation_ingress|native_sensor_messages|native_atlas_localization|native_observation_source|reader_retirement)\\.cpp$")

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
    autolink
    automsgs
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
    autonomy::cli11
)

if((BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND) OR (BUILD_TENSORRT AND TensorRT_FOUND))
  autocmake_library(perception_inference SHARED NO_EXPORT
    SOURCES inference/model_session.cpp inference/image_input.cpp inference/tensor_bundle.cpp
    INCLUDES ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
    DEPENDENCIES autonomy_common autonomy_proto
      $<TARGET_NAME_IF_EXISTS:autonomy_opencv>)
  autocmake_library(perception_core SHARED NO_EXPORT
    SOURCES engine/engine_component.cpp engine/task_pipeline.cpp engine/options.cpp
      engine/binding.cpp models/model_registry.cpp tasks/task_registry.cpp
      models/yolo/yolo_model.cpp models/segformer/segformer_model.cpp
      models/sam2/sam2_model.cpp models/flashocc/flashocc_model.cpp
      models/boxer/boxer_model.cpp tasks/lifting/lifting_task.cpp
      tasks/scene_graph/room_geometry.cpp tasks/scene_graph/room_attribution.cpp tasks/room_segmentation/room_segmentation_task.cpp
      tasks/detection/detection_task.cpp tasks/instance_seg/instance_seg_task.cpp
      tasks/semantic_seg/semantic_seg_task.cpp tasks/classification/classification_task.cpp
      tasks/keypoint/keypoint_task.cpp tasks/obb/obb_task.cpp
      behaviors/tracking/association.cpp tasks/depth/depth_task.cpp models/moge/moge_model.cpp
    INCLUDES ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
    PRIVATE_DEPENDENCIES perception_inference autonomy autolink automsgs)
  autocmake_binary(autonomy.perception_object NO_EXPORT
    SOURCES engine/object_inference_main.cpp
    DEPENDENCIES perception_core autonomy_proto autolink)
  set_target_properties(perception_core PROPERTIES OUTPUT_NAME "base_component")
  if(BUILD_TEST OR BUILD_TESTING)
    autocmake_test(autonomy.perception.base.component_test
      SOURCES engine/engine_component_test.cpp engine/options_test.cpp
      DEPENDENCIES perception_core)
  endif()
endif()

autocmake_library(perception_behaviors SHARED NO_EXPORT
  SOURCES behaviors/follow/follow_component.cpp behaviors/follow/options.cpp behaviors/follow/localizer.cpp
    behaviors/follow/grid.cpp behaviors/follow/planner.cpp
  INCLUDES ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
  PRIVATE_DEPENDENCIES autonomy autolink automsgs)

set_target_properties(perception_behaviors PROPERTIES OUTPUT_NAME "follow_component")

install(FILES data/boundary.ply DESTINATION share/${PROJECT_NAME}/perception/data)

# --- install -----------------------------------------------------------------
autocmake_install_share(perception
  CONF_PATTERN "*.pb.txt"
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")

install(PROGRAMS tools/depth_completion.py tools/person_reid.py
  DESTINATION share/${PROJECT_NAME}/perception/tools)
install(FILES tools/demo_config.py tools/image_message.py
  DESTINATION share/${PROJECT_NAME}/perception/tools)
