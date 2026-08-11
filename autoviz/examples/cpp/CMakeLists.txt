# Autoviz C++ tutorial publishers (01–24).
#
# Targets: autoviz_cpp_<name>  →  ${CMAKE_BINARY_DIR}/bin/examples/
# Meta:    autoviz_cpp_examples (builds all); hooked as dependency of `autoviz`.

if(NOT TARGET autolink OR NOT TARGET automsgs)
  message(STATUS "Autoviz C++ examples skipped (autolink/automsgs not available)")
  return()
endif()

if(NOT AUTOVIZ_BUILD_CPP_EXAMPLES)
  message(STATUS "Autoviz C++ examples disabled (AUTOVIZ_BUILD_CPP_EXAMPLES=OFF)")
  return()
endif()

set(AUTOVIZ_CPP_EXAMPLES_OUT "${CMAKE_BINARY_DIR}/bin/examples")
set(_AUTOVIZ_CPP_EXAMPLE_TARGETS "")

function(autoviz_cpp_tutorial SHORT_NAME SRC)
  set(target "autoviz_cpp_${SHORT_NAME}")
  add_executable(${target} ${SRC})
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${target} PRIVATE autolink automsgs)
  if(TARGET glog::glog)
    target_link_libraries(${target} PRIVATE glog::glog)
  endif()
  set_target_properties(${target} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${AUTOVIZ_CPP_EXAMPLES_OUT}")
  target_compile_features(${target} PRIVATE cxx_std_17)
  set(_AUTOVIZ_CPP_EXAMPLE_TARGETS "${_AUTOVIZ_CPP_EXAMPLE_TARGETS};${target}"
    PARENT_SCOPE)
endfunction()

autoviz_cpp_tutorial(01_poses 01_tutorial_poses.cpp)
autoviz_cpp_tutorial(02_occupancy_grid 02_tutorial_occupancy_grid.cpp)
autoviz_cpp_tutorial(03_image 03_tutorial_image.cpp)
autoviz_cpp_tutorial(04_sensor 04_tutorial_sensor.cpp)
autoviz_cpp_tutorial(05_tf 05_tutorial_transform_tree.cpp)
autoviz_cpp_tutorial(06_urdf 06_tutorial_urdf.cpp)
autoviz_cpp_tutorial(07_marker 07_tutorial_visualization_marker.cpp)
autoviz_cpp_tutorial(08_force 08_tutorial_force_joint.cpp)
autoviz_cpp_tutorial(09_plot 09_tutorial_plot.cpp)
autoviz_cpp_tutorial(10_log 10_tutorial_log.cpp)
autoviz_cpp_tutorial(11_state 11_tutorial_state_transitions.cpp)
autoviz_cpp_tutorial(12_channel_graph 12_tutorial_channel_graph.cpp)
autoviz_cpp_tutorial(13_raw 13_tutorial_raw_messages.cpp)
autoviz_cpp_tutorial(14_table 14_tutorial_table.cpp)
autoviz_cpp_tutorial(15_diagnostics 15_tutorial_diagnostics.cpp)
autoviz_cpp_tutorial(16_parameters 16_tutorial_parameters.cpp)
autoviz_cpp_tutorial(17_visions 17_tutorial_visions.cpp)
autoviz_cpp_tutorial(18_robot_all 18_tutorial_robot_all_data_sim.cpp)
autoviz_cpp_tutorial(19_geometry 19_tutorial_geometry_extras.cpp)
autoviz_cpp_tutorial(20_scalar 20_tutorial_scalar_env.cpp)
autoviz_cpp_tutorial(21_audio 21_tutorial_audio.cpp)
autoviz_cpp_tutorial(22_service 22_tutorial_service.cpp)
autoviz_cpp_tutorial(23_map_gps 23_tutorial_map_gps.cpp)
autoviz_cpp_tutorial(24_teleop_echo 24_tutorial_teleop_echo.cpp)
autoviz_cpp_tutorial(26_raw_pub 26_test_rawmessage_pub.cpp)

list(REMOVE_ITEM _AUTOVIZ_CPP_EXAMPLE_TARGETS "")
add_custom_target(autoviz_cpp_examples DEPENDS ${_AUTOVIZ_CPP_EXAMPLE_TARGETS})

# Building `autoviz` also builds tutorial publishers (same as default `all`).
if(TARGET autoviz)
  add_dependencies(autoviz autoviz_cpp_examples)
endif()

list(LENGTH _AUTOVIZ_CPP_EXAMPLE_TARGETS _autoviz_cpp_n)
message(STATUS "Autoviz C++ examples: ${_autoviz_cpp_n} targets → ${AUTOVIZ_CPP_EXAMPLES_OUT}")
message(STATUS "  cmake --build build --target autoviz_cpp_examples")
