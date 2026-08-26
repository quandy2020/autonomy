# grid_map_demos — ROS-free ports of ANYbotics grid_map_demos.
#
# Built when BUILD_GRID_MAP_DEMOS=ON.

set(_GRID_MAP_DEMOS_ROOT
  "${PROJECT_SOURCE_DIR}/autonomy/map/grid_map/grid_map_demos")

function(grid_map_demo NAME)
  cmake_parse_arguments(ARG "" "" "SRCS" ${ARGN})
  google_binary(${NAME} SRCS ${ARG_SRCS})
  target_include_directories(${NAME} PRIVATE "${_GRID_MAP_DEMOS_ROOT}")
endfunction()

grid_map_demo(grid_map_simple_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/simple_demo.cpp)
grid_map_demo(grid_map_tutorial_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/tutorial_demo.cpp)
grid_map_demo(grid_map_opencv_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/opencv_demo.cpp)
grid_map_demo(grid_map_move_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/move_demo.cpp)
grid_map_demo(grid_map_resolution_change_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/resolution_change_demo.cpp)
grid_map_demo(grid_map_iterators_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/iterators_demo.cpp)
grid_map_demo(grid_map_filters_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/filters_demo.cpp)
grid_map_demo(grid_map_image_to_gridmap_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/image_to_gridmap_demo.cpp)
grid_map_demo(grid_map_gridmap_to_image_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/grid_map_to_image_demo.cpp)
grid_map_demo(grid_map_sdf_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/sdf_demo.cpp)
grid_map_demo(grid_map_normal_filter_comparison_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/normal_filter_comparison_demo.cpp)
grid_map_demo(grid_map_iterator_benchmark
  SRCS ${_GRID_MAP_DEMOS_ROOT}/iterator_benchmark.cpp)
grid_map_demo(grid_map_interpolation_demo
  SRCS ${_GRID_MAP_DEMOS_ROOT}/interpolation_demo.cpp)

install(DIRECTORY
  ${_GRID_MAP_DEMOS_ROOT}/config
  ${_GRID_MAP_DEMOS_ROOT}/data
  DESTINATION share/autonomy/map/grid_map/grid_map_demos
)

unset(_GRID_MAP_DEMOS_ROOT)
