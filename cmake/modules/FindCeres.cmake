# Copyright 2024 The OpenRobotic Beginner Authors
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

# FindCeres.cmake

# Locate Ceres library
# This module defines the following variables:
#  CERES_FOUND - True if the library was found
#  CERES_INCLUDE_DIRS - Include directories for Ceres
#  CERES_LIBRARIES - Libraries to link against

find_path(CERES_INCLUDE_DIR
  NAMES ceres/ceres.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

find_library(CERES_LIBRARY
  NAMES ceres
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_PREFIX_PATH}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ceres DEFAULT_MSG
  CERES_INCLUDE_DIR CERES_LIBRARY)

if (CERES_FOUND)
  set(CERES_INCLUDE_DIRS ${CERES_INCLUDE_DIR})
  set(CERES_LIBRARIES ${CERES_LIBRARY})
endif()
