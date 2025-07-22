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

# FindOSQP.cmake
# This module looks for the OSQP library and headers
# Once done, it defines
#   OSQP_FOUND - system has OSQP
#   OSQP_INCLUDE_DIRS - the OSQP include directories
#   OSQP_LIBRARIES - the libraries needed to use OSQP

find_path(OSQP_INCLUDE_DIR
  NAMES osqp/osqp.h
  PATHS ${CMAKE_INSTALL_PREFIX}/include /usr/local/include /usr/include
)

find_library(OSQP_LIBRARY
  NAMES osqp
  PATHS ${CMAKE_INSTALL_PREFIX}/lib /usr/local/lib /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OSQP DEFAULT_MSG OSQP_LIBRARY OSQP_INCLUDE_DIR)

if(OSQP_FOUND)
  set(OSQP_LIBRARIES ${OSQP_LIBRARY})
  set(OSQP_INCLUDE_DIRS ${OSQP_INCLUDE_DIR})
endif()

mark_as_advanced(OSQP_INCLUDE_DIR OSQP_LIBRARY)