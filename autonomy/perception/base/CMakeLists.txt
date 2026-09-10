# Build Base as a loadable autolink component only when an inference backend
# and the component runtime are available. Registration stays on this DSO.
if(NOT TARGET autolink)
  return()
endif()

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

add_library(base_component SHARED
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
target_include_directories(base_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(base_component PRIVATE autonomy autolink automsgs)
set_target_properties(base_component PROPERTIES OUTPUT_NAME base_component)

if(BUILD_TEST)
  set(_base_component_test autonomy.perception.base.component_test)
  google_test(${_base_component_test} base_component_test.cpp)
  add_dependencies(${_base_component_test} base_component)
  target_link_libraries(${_base_component_test} PRIVATE base_component)
  unset(_base_component_test)
endif()
