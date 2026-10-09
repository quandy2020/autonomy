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

# -----------------------------------------------------------------------------
# autonomy/map — costmap / grid_map and map server utilities
#
# Builds
#   autonomy_map / autonomy::autonomy_map
#   autonomy_opencv   INTERFACE (shared with localization / perception)
#
# Toggle
#   AUTONOMY_BUILD_MAP
# -----------------------------------------------------------------------------

# --- dependencies ------------------------------------------------------------
# Domain finds run after root autocmake_build(); REQUIRED must FATAL locally.
autocmake_find(OpenCV REQUIRED PRIVATE)
autocmake_find(PCL REQUIRED PRIVATE
  COMPONENTS common features filters io kdtree segmentation surface)
autocmake_find(TBB REQUIRED PRIVATE)
autocmake_find(OpenMP PRIVATE QUIET)

foreach(_pkg IN ITEMS OpenCV PCL TBB)
  if(NOT ${_pkg}_FOUND)
    message(FATAL_ERROR "map: ${_pkg} is required")
  endif()
endforeach()

if(NOT TARGET autonomy_opencv)
  add_library(autonomy_opencv INTERFACE)
  target_include_directories(autonomy_opencv SYSTEM INTERFACE
    $<BUILD_INTERFACE:${OpenCV_INCLUDE_DIRS}>)
  target_link_libraries(autonomy_opencv INTERFACE ${OpenCV_LIBS})
endif()

# Eigen plugins used by grid_map_core (must be unique project-wide).
set(_eigen_functors_plugin
  "${CMAKE_CURRENT_SOURCE_DIR}/grid_map/grid_map_core/eigen_plugins/functors_plugin.hpp")
set(_eigen_densebase_plugin
  "${CMAKE_CURRENT_SOURCE_DIR}/grid_map/grid_map_core/eigen_plugins/dense_base_plugin.hpp")
if(EIGEN_FUNCTORS_PLUGIN AND NOT EIGEN_FUNCTORS_PLUGIN STREQUAL _eigen_functors_plugin)
  message(FATAL_ERROR "EIGEN_FUNCTORS_PLUGIN already defined!")
endif()
if(EIGEN_DENSEBASE_PLUGIN AND NOT EIGEN_DENSEBASE_PLUGIN STREQUAL _eigen_densebase_plugin)
  message(FATAL_ERROR "EIGEN_DENSEBASE_PLUGIN already defined!")
endif()

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_MAP_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_MAP_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_MAP_SRCS EXCLUDE REGEX "_test\\.cpp$")
list(FILTER AUTONOMY_MAP_SRCS EXCLUDE REGEX "/grid_map_demos/")
list(FILTER AUTONOMY_MAP_SRCS EXCLUDE REGEX "/strata/")
list(FILTER AUTONOMY_MAP_SRCS EXCLUDE REGEX "/(fake_|mock_)")
list(FILTER AUTONOMY_MAP_SRCS EXCLUDE REGEX "test_helpers")
file(GLOB _mock_filter
  "${CMAKE_CURRENT_SOURCE_DIR}/grid_map/grid_map_filters/mock_filter.cpp")
list(APPEND AUTONOMY_MAP_SRCS ${_mock_filter})

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_map
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_MAP_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_opencv)
add_library(autonomy::map ALIAS autonomy_map)

target_include_directories(autonomy_map SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(autonomy_map PUBLIC
  ${PCL_LIBRARIES}
  $<TARGET_NAME_IF_EXISTS:OpenMP::OpenMP_CXX>)
target_compile_definitions(autonomy_map PUBLIC
  ${PCL_DEFINITIONS}
  $<$<BOOL:${OpenMP_CXX_FOUND}>:GRID_MAP_PCL_OPENMP_FOUND=1>
  "EIGEN_FUNCTORS_PLUGIN=\"${_eigen_functors_plugin}\""
  "EIGEN_DENSEBASE_PLUGIN=\"${_eigen_densebase_plugin}\"")

# --- install -----------------------------------------------------------------
autocmake_install_share(map CONF)
