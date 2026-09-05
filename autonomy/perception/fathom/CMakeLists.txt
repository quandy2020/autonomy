# Fathom is a loadable autolink module. Keep its entrypoint out of libautonomy
# so mainboard owns the component registration in this shared object.
if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND OR NOT TARGET autolink)
  return()
endif()

add_library(fathom_component SHARED component.cpp)
target_include_directories(fathom_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(fathom_component PRIVATE autonomy autolink automsgs)
set_target_properties(fathom_component PROPERTIES OUTPUT_NAME fathom_component)

if(BUILD_TEST)
  set(_fathom_component_test autonomy.perception.fathom.component_test)
  google_test(${_fathom_component_test} component_test.cpp)
  add_dependencies(${_fathom_component_test} fathom_component)
  target_link_libraries(${_fathom_component_test} PRIVATE fathom_component)
  unset(_fathom_component_test)
endif()
