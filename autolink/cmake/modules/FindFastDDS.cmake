# Copyright 2025 The Openbot Authors
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

# Find Fast DDS (aka Fast-DDS, eProsima)
# This module sets the following variables on success:
#   FastDDS_FOUND
#   FastDDS_INCLUDE_DIRS
#   FastDDS_LIBRARIES       (includes both fastdds and fastcdr)
#   FastDDS_fastdds_LIBRARY
#   FastDDS_fastcdr_LIBRARY

include(FindPackageHandleStandardArgs)

# Try CMake config packages first (preferred when provided by system). Use CONFIG
# to avoid recursively invoking this Find module.
find_package(fastdds CONFIG QUIET)
find_package(fastcdr CONFIG QUIET)

if (TARGET fastdds AND TARGET fastcdr)
  # Both imported targets are available
  set(FastDDS_FOUND TRUE)
  # Try to extract include dirs if the packages export variables
  set(FastDDS_INCLUDE_DIRS "${fastdds_INCLUDE_DIRS};${fastcdr_INCLUDE_DIRS}")
  set(FastDDS_LIBRARIES fastdds fastcdr)
  set(FastDDS_fastdds_LIBRARY fastdds)
  set(FastDDS_fastcdr_LIBRARY fastcdr)
else()
  # Fallback: manual search in common prefixes (e.g., /opt/cyber)
  find_path(FastDDS_INCLUDE_DIR
    NAMES fastdds/dds/domain/DomainParticipantFactory.hpp
    PATHS /usr /usr/local /opt/cyber
    PATH_SUFFIXES include
  )

  find_library(FastDDS_fastdds_LIBRARY
    NAMES fastdds
    PATHS /usr /usr/local /opt/cyber
    PATH_SUFFIXES lib
  )

  find_library(FastDDS_fastcdr_LIBRARY
    NAMES fastcdr
    PATHS /usr /usr/local /opt/cyber
    PATH_SUFFIXES lib
  )

  if (FastDDS_INCLUDE_DIR AND FastDDS_fastdds_LIBRARY AND FastDDS_fastcdr_LIBRARY)
    set(FastDDS_FOUND TRUE)
    set(FastDDS_INCLUDE_DIRS ${FastDDS_INCLUDE_DIR})
    set(FastDDS_LIBRARIES ${FastDDS_fastdds_LIBRARY} ${FastDDS_fastcdr_LIBRARY})
  else()
    set(FastDDS_FOUND FALSE)
  endif()
endif()

find_package_handle_standard_args(FastDDS DEFAULT_MSG FastDDS_FOUND)

mark_as_advanced(FastDDS_INCLUDE_DIR FastDDS_fastdds_LIBRARY FastDDS_fastcdr_LIBRARY)


