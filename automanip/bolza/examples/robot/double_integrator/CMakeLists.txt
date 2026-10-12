# Double-integrator terminal OCP. Subdirectory of examples.
# Links libbolza and glog.

find_package(GLog REQUIRED)

add_executable(bolza_double_integrator
  "${CMAKE_CURRENT_LIST_DIR}/double_integrator.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(bolza_double_integrator PRIVATE bolza GLog::GLog)
get_filename_component(BOLZA_EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(bolza_double_integrator PRIVATE "${BOLZA_EXAMPLES_DIR}")
set_target_properties(bolza_double_integrator PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)

install(TARGETS bolza_double_integrator RUNTIME DESTINATION bin)
