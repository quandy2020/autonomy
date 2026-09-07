# Build Shadow as a loadable autolink component only when its concrete
# inference backend and component runtime are available. Registration remains
# owned by this shared object rather than libautonomy.
if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND OR NOT TARGET autolink)
  return()
endif()

add_library(shadow_component SHARED shadow_component.cpp)
target_include_directories(shadow_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(shadow_component PRIVATE autonomy autolink automsgs)
set_target_properties(shadow_component PROPERTIES OUTPUT_NAME shadow_component)

if(BUILD_TEST)
  set(_shadow_component_test autonomy.perception.shadow.component_test)
  google_test(${_shadow_component_test} shadow_component_test.cpp)
  add_dependencies(${_shadow_component_test} shadow_component)
  target_link_libraries(${_shadow_component_test} PRIVATE shadow_component)
  unset(_shadow_component_test)
endif()
