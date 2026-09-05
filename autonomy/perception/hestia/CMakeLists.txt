# Build Hestia as a loadable autolink component only when its concrete
# inference backend and component runtime are available. The entrypoint stays
# outside libautonomy so registration is owned by this shared object.
if(NOT BUILD_ONNXRUNTIME OR NOT OnnxRuntime_FOUND OR NOT TARGET autolink)
  return()
endif()

add_library(hestia_component SHARED hestia_component.cpp)
target_include_directories(hestia_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(hestia_component PRIVATE autonomy autolink automsgs)
set_target_properties(hestia_component PROPERTIES OUTPUT_NAME hestia_component)

if(BUILD_TEST)
  set(_hestia_component_test autonomy.perception.hestia.component_test)
  google_test(${_hestia_component_test} hestia_component_test.cpp)
  add_dependencies(${_hestia_component_test} hestia_component)
  target_link_libraries(${_hestia_component_test} PRIVATE hestia_component)
  unset(_hestia_component_test)
endif()
