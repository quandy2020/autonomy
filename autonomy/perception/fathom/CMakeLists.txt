# Build Fathom as a loadable autolink component when an inference backend and
# autolink are available. Registration lives in this shared object, not
# libautonomy.
if(NOT TARGET autolink)
  return()
endif()

set(_fathom_backend FALSE)
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  set(_fathom_backend TRUE)
endif()
if(BUILD_TENSORRT AND TensorRT_FOUND)
  set(_fathom_backend TRUE)
endif()
if(NOT _fathom_backend)
  return()
endif()
unset(_fathom_backend)

add_library(fathom_component SHARED fathom_component.cpp)
target_include_directories(fathom_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(fathom_component PRIVATE autonomy autolink automsgs)
set_target_properties(fathom_component PROPERTIES OUTPUT_NAME fathom_component)

if(BUILD_TEST)
  set(_fathom_component_test autonomy.perception.fathom.component_test)
  google_test(${_fathom_component_test} fathom_component_test.cpp)
  add_dependencies(${_fathom_component_test} fathom_component)
  target_link_libraries(${_fathom_component_test} PRIVATE fathom_component)
  unset(_fathom_component_test)
endif()
