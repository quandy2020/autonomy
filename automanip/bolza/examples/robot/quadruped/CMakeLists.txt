# Quadruped terminal OCP. Subdirectory of examples.
# Links libbolza and glog.

find_package(GLog REQUIRED)

add_executable(bolza_quadruped
  "${CMAKE_CURRENT_LIST_DIR}/quadruped.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(bolza_quadruped PRIVATE bolza GLog::GLog)
get_filename_component(BOLZA_EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(bolza_quadruped PRIVATE "${BOLZA_EXAMPLES_DIR}")
set_target_properties(bolza_quadruped PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)

install(TARGETS bolza_quadruped RUNTIME DESTINATION bin)
