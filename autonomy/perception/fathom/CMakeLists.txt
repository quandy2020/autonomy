# Build Fathom as a loadable autolink component only when its concrete
# inference backend and component runtime are available. The entrypoint stays
# outside libautonomy so registration is owned by this shared object.
if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND OR NOT TARGET autolink)
  return()
endif()

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
