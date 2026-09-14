# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# autonomy/map — costmap / grid_map and map server utilities

include(autonomy_module)
autonomy_module(map)

# Eigen plugins required by grid_map_core (propagate PUBLIC to dependents).
set(_eigen_functors_plugin
  "${CMAKE_CURRENT_SOURCE_DIR}/grid_map/grid_map_core/eigen_plugins/functors_plugin.hpp")
set(_eigen_densebase_plugin
  "${CMAKE_CURRENT_SOURCE_DIR}/grid_map/grid_map_core/eigen_plugins/dense_base_plugin.hpp")
if(EIGEN_FUNCTORS_PLUGIN)
  if(NOT EIGEN_FUNCTORS_PLUGIN STREQUAL _eigen_functors_plugin)
    message(FATAL_ERROR "EIGEN_FUNCTORS_PLUGIN already defined!")
  endif()
endif()
if(EIGEN_DENSEBASE_PLUGIN)
  if(NOT EIGEN_DENSEBASE_PLUGIN STREQUAL _eigen_densebase_plugin)
    message(FATAL_ERROR "EIGEN_DENSEBASE_PLUGIN already defined!")
  endif()
endif()

# Library sources: GLOB *.cpp; drop mains/tests, demos, strata, and test fakes.
autonomy_glob_srcs(_MAP_SRCS)
list(FILTER _MAP_SRCS EXCLUDE REGEX "/grid_map_demos/")
list(FILTER _MAP_SRCS EXCLUDE REGEX "/strata/")
list(FILTER _MAP_SRCS EXCLUDE REGEX "/(fake_|mock_)")
list(FILTER _MAP_SRCS EXCLUDE REGEX "test_helpers")
# Production grid_map filters still need mock_filter.cpp.
file(GLOB _mock_filter
  "${CMAKE_CURRENT_SOURCE_DIR}/grid_map/grid_map_filters/mock_filter.cpp")
list(APPEND _MAP_SRCS ${_mock_filter})

autonomy_library(
  DEPENDENCIES autonomy_common autonomy_transform
  FEATURES pcl
  SRCS ${_MAP_SRCS}
)
target_compile_definitions(autonomy_map PUBLIC
  "EIGEN_FUNCTORS_PLUGIN=\"${_eigen_functors_plugin}\""
  "EIGEN_DENSEBASE_PLUGIN=\"${_eigen_densebase_plugin}\"")
unset(_MAP_SRCS)
unset(_mock_filter)
unset(_eigen_functors_plugin)
unset(_eigen_densebase_plugin)

# Optional ROS-free grid_map demos (BUILD_GRID_MAP_DEMOS).
if(BUILD_GRID_MAP_DEMOS)
  add_subdirectory(grid_map/grid_map_demos)
endif()

# Runtime conf (maps / simulation)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/map/conf
)
