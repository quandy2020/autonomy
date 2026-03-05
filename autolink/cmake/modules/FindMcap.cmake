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

# FindMcap.cmake
#
# Locate MCAP library (header-only)
# This module defines the following variables:
#  MCAP_FOUND - True if the library was found
#  MCAP_INCLUDE_DIRS - Include directories for MCAP
#  MCAP_LIBRARIES - Libraries to link against (empty for header-only, but includes dependencies)
#
# MCAP is a header-only library that optionally depends on:
#  - lz4 (optional, can be disabled with MCAP_COMPRESSION_NO_LZ4)
#  - zstd (optional, can be disabled with MCAP_COMPRESSION_NO_ZSTD)

# Find MCAP include directory
find_path(MCAP_INCLUDE_DIR
  NAMES mcap/mcap.hpp
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

# Try to find optional dependencies (lz4 and zstd)
# These are not required for MCAP to work, but are commonly used
find_library(LZ4_LIBRARY
  NAMES lz4
  PATHS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    ${CMAKE_PREFIX_PATH}/lib
)

find_library(ZSTD_LIBRARY
  NAMES zstd
  PATHS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    ${CMAKE_PREFIX_PATH}/lib
)

# Check for header-only library availability
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Mcap DEFAULT_MSG
  MCAP_INCLUDE_DIR)

# Set the results
if(MCAP_FOUND)
  set(MCAP_INCLUDE_DIRS ${MCAP_INCLUDE_DIR})
  
  # Collect optional dependency libraries
  set(MCAP_LIBRARIES)
  if(LZ4_LIBRARY)
    list(APPEND MCAP_LIBRARIES ${LZ4_LIBRARY})
  endif()
  if(ZSTD_LIBRARY)
    list(APPEND MCAP_LIBRARIES ${ZSTD_LIBRARY})
  endif()
  
  # If no optional libraries found, set to empty (header-only is still valid)
  if(NOT MCAP_LIBRARIES)
    set(MCAP_LIBRARIES "")
  endif()
endif()

mark_as_advanced(MCAP_INCLUDE_DIR LZ4_LIBRARY ZSTD_LIBRARY)
