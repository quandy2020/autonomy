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

# @file FindOSQP.cmake
# @brief Find-module for the OSQP quadratic-programming library.
#
# @var OSQP_FOUND         Whether OSQP was found
# @var OSQP_INCLUDE_DIRS  Include directories
# @var OSQP_LIBRARIES     Libraries to link
# @creates OSQP::OSQP     IMPORTED target (when found)

find_path(OSQP_INCLUDE_DIR
  NAMES osqp/osqp.h
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /opt/homebrew/include
    /usr/local/include
    /usr/include
)

find_library(OSQP_LIBRARY
  NAMES osqp
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /opt/homebrew/lib
    /usr/local/lib
    /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OSQP DEFAULT_MSG OSQP_LIBRARY OSQP_INCLUDE_DIR)

if(OSQP_FOUND)
  set(OSQP_LIBRARIES ${OSQP_LIBRARY})
  set(OSQP_INCLUDE_DIRS ${OSQP_INCLUDE_DIR})
  if(NOT TARGET OSQP::OSQP)
    add_library(OSQP::OSQP UNKNOWN IMPORTED)
    set_target_properties(OSQP::OSQP PROPERTIES
      IMPORTED_LOCATION "${OSQP_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${OSQP_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(OSQP_INCLUDE_DIR OSQP_LIBRARY)
