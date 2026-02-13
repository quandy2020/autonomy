# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

# FindBox2D.cmake

# Locate BOX2D library
# This module defines the following variables:
#  BOX2D_FOUND - True if the library was found
#  BOX2D_INCLUDE_DIRS - Include directories for BOX2D
#  BOX2D_LIBRARIES - Libraries to link against

find_path(BOX2D_INCLUDE_DIR
  NAMES box2d/box2d.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

find_library(BOX2D_LIBRARY
  NAMES box2d
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_PREFIX_PATH}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Box2D DEFAULT_MSG
  BOX2D_INCLUDE_DIR BOX2D_LIBRARY)

if (BOX2D_FOUND)
  set(CERES_INCLUDE_DIRS ${BOX2D_INCLUDE_DIR})
  set(CERES_LIBRARIES ${BOX2D_LIBRARY})
endif()
