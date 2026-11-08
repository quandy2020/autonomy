# Copyright 2018 The Cartographer Authors
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

# Prefer Eigen's package configuration when GTSAM requests Eigen in module mode.
if(TARGET Eigen3::Eigen)
  set(Eigen3_FOUND TRUE)
  set(EIGEN3_FOUND TRUE)
  return()
endif()

find_package(Eigen3 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  set(Eigen3_FOUND TRUE)
  set(EIGEN3_FOUND TRUE)
  return()
endif()

find_path(EIGEN3_INCLUDE_DIR
  NAMES Eigen/Core
  PATH_SUFFIXES eigen3
  PATHS
    /usr/local/include
    /usr/include
)

if(EIGEN3_INCLUDE_DIR AND EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  set(Eigen3_FOUND TRUE)
  set(EIGEN3_FOUND TRUE)
  set(Eigen3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
  set(EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
  if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
  endif()
else()
  set(Eigen3_FOUND FALSE)
  set(EIGEN3_FOUND FALSE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
  REQUIRED_VARS EIGEN3_INCLUDE_DIR
)
