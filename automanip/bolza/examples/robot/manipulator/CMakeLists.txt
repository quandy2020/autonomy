# Manipulator terminal OCP. Subdirectory of examples.
# Links libbolza and glog.

find_package(GLog REQUIRED)

add_executable(bolza_manipulator
  "${CMAKE_CURRENT_LIST_DIR}/manipulator.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
target_link_libraries(bolza_manipulator PRIVATE bolza GLog::GLog)
get_filename_component(BOLZA_EXAMPLES_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
target_include_directories(bolza_manipulator PRIVATE "${BOLZA_EXAMPLES_DIR}")
set_target_properties(bolza_manipulator PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)

install(TARGETS bolza_manipulator RUNTIME DESTINATION bin)
