# Quadrotor terminal OCP. Subdirectory of examples.
# Links libbolza and glog.

find_package(GLog REQUIRED)

add_executable(bolza_quadrotor
  "${CMAKE_CURRENT_LIST_DIR}/quadrotor.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(bolza_quadrotor PRIVATE bolza GLog::GLog)
get_filename_component(BOLZA_EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(bolza_quadrotor PRIVATE "${BOLZA_EXAMPLES_DIR}")
set_target_properties(bolza_quadrotor PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)

install(TARGETS bolza_quadrotor RUNTIME DESTINATION bin)
