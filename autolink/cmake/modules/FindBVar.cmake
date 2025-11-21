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

# Locate the BVar library
# BVar is a statistics library providing LatencyRecorder, Adder, Status, etc.

# Find bvar include directory
find_path(BVAR_INCLUDE_DIR
  NAMES third_party/var/bvar/bvar.h
  PATHS /usr/local/include
        /usr/include
        ${CMAKE_INSTALL_PREFIX}/include
)

# Find bvar library
find_library(BVAR_LIBRARY
  NAMES bvar
  PATHS /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
        ${CMAKE_INSTALL_PREFIX}/lib
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu
)

# Set the results
if(BVAR_INCLUDE_DIR AND BVAR_LIBRARY)
  set(BVAR_FOUND TRUE)
  set(BVAR_INCLUDE_DIRS ${BVAR_INCLUDE_DIR})
  set(BVAR_LIBRARIES ${BVAR_LIBRARY})
else()
  set(BVAR_FOUND FALSE)
  set(BVAR_INCLUDE_DIRS)
  set(BVAR_LIBRARIES)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BVar
  DEFAULT_MSG
  BVAR_LIBRARIES
  BVAR_INCLUDE_DIRS
)

# Set standard variables for compatibility (mixed case matching package name)
if(BVar_FOUND)
  set(BVar_LIBRARIES ${BVAR_LIBRARIES})
  set(BVar_INCLUDE_DIRS ${BVAR_INCLUDE_DIRS})
endif()

mark_as_advanced(
  BVAR_INCLUDE_DIR
  BVAR_INCLUDE_DIRS
  BVAR_LIBRARY
  BVAR_LIBRARIES
)

