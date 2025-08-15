# Copyright 2024 The Openbot Authors (duyongquan)
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

set(EIGEN_FUNCTORS_PLUGIN_PATH "${PROJECT_SOURCE_DIR}/autonomy/map/grid_map/grid_map_core/eigen_plugins/functors_plugin.hpp")
if (EIGEN_FUNCTORS_PLUGIN)
  if (NOT EIGEN_FUNCTORS_PLUGIN STREQUAL EIGEN_FUNCTORS_PLUGIN_PATH)
    MESSAGE(FATAL_ERROR "EIGEN_FUNCTORS_PLUGIN already defined!")
  endif ()
else (EIGEN_FUNCTORS_PLUGIN)
  add_definitions(-DEIGEN_FUNCTORS_PLUGIN=\"${EIGEN_FUNCTORS_PLUGIN_PATH}\")
endif (EIGEN_FUNCTORS_PLUGIN)

set(EIGEN_DENSEBASE_PLUGIN_PATH "${PROJECT_SOURCE_DIR}/autonomy/map/grid_map/grid_map_core/eigen_plugins/dense_base_plugin.hpp")
if (EIGEN_DENSEBASE_PLUGIN)
  if (NOT EIGEN_DENSEBASE_PLUGIN STREQUAL EIGEN_DENSEBASE_PLUGIN_PATH)
    MESSAGE(FATAL_ERROR "EIGEN_DENSEBASE_PLUGIN already defined!")
  endif ()
else (EIGEN_DENSEBASE_PLUGIN)
    add_definitions(-DEIGEN_DENSEBASE_PLUGIN=\"${EIGEN_DENSEBASE_PLUGIN_PATH}\")
endif (EIGEN_DENSEBASE_PLUGIN)


