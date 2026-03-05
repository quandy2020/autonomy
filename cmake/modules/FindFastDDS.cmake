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

# In ROS, the library is typically libfastrtps.so (not libfastdds.so); prefer
# module search so we get the actual library path and avoid "cannot find -lfastdds".
if(DEFINED ENV{ROS_DISTRO})
  set(_FastDDS_skip_config TRUE)
endif()

# Try CMake config packages first (when not in ROS). Use CONFIG to avoid
# recursively invoking this Find module.
if(NOT _FastDDS_skip_config)
  find_package(fastdds CONFIG QUIET)
  find_package(fastcdr CONFIG QUIET)
endif()

if (NOT _FastDDS_skip_config AND TARGET fastdds AND TARGET fastcdr)
  # Both imported targets are available
  set(FastDDS_FOUND TRUE)
  # Try to extract include dirs if the packages export variables
  set(FastDDS_INCLUDE_DIRS "${fastdds_INCLUDE_DIRS};${fastcdr_INCLUDE_DIRS}")
  set(FastDDS_LIBRARIES fastdds fastcdr)
  set(FastDDS_fastdds_LIBRARY fastdds)
  set(FastDDS_fastcdr_LIBRARY fastcdr)
else()
  # Get ROS installation directory if available
  if(DEFINED ENV{ROS_DISTRO})
    get_filename_component(_ROS_ROOT "/opt/ros/$ENV{ROS_DISTRO}" ABSOLUTE)
  else()
    # Try common ROS installation paths
    if(EXISTS "/opt/ros/jazzy")
      set(_ROS_ROOT "/opt/ros/jazzy")
    elseif(EXISTS "/opt/ros/humble")
      set(_ROS_ROOT "/opt/ros/humble")
    elseif(EXISTS "/opt/ros/foxy")
      set(_ROS_ROOT "/opt/ros/foxy")
    else()
      set(_ROS_ROOT "")
    endif()
  endif()

  # Search for FastDDS include directory
  # Check both standard path (fastdds/dds/...) and ROS path (fastrtps/fastdds/dds/...)
  find_path(FastDDS_INCLUDE_DIR
    NAMES fastdds/dds/domain/DomainParticipantFactory.hpp
    PATHS 
      ${_ROS_ROOT}
      /usr 
      /usr/local 
    PATH_SUFFIXES include
  )

  # Also check ROS-specific path pattern
  if(NOT FastDDS_INCLUDE_DIR AND _ROS_ROOT)
    find_path(FastDDS_INCLUDE_DIR
      NAMES fastrtps/fastdds/dds/domain/DomainParticipantFactory.hpp
      PATHS ${_ROS_ROOT}
      PATH_SUFFIXES include
    )
    if(FastDDS_INCLUDE_DIR)
      # Adjust include directory to point to include parent
      # so includes work as fastrtps/fastdds/...
      set(FastDDS_INCLUDE_DIR "${FastDDS_INCLUDE_DIR}")
    endif()
  endif()

  # Search for FastDDS library (fastdds or fastrtps in ROS)
  find_library(FastDDS_fastdds_LIBRARY
    NAMES fastdds fastrtps
    PATHS 
      ${_ROS_ROOT}
      /usr 
      /usr/local 
    PATH_SUFFIXES lib
  )

  # Ensure fastcdr is found from the same location as fastdds
  # If fastdds is in /usr/local, prefer fastcdr from /usr/local
  if (FastDDS_fastdds_LIBRARY MATCHES "/usr/local")
    find_library(FastDDS_fastcdr_LIBRARY
      NAMES fastcdr
      PATHS /usr/local
      PATH_SUFFIXES lib
      NO_DEFAULT_PATH
    )
  endif()
  
  # Fallback to general search if not found above
  if (NOT FastDDS_fastcdr_LIBRARY)
    find_library(FastDDS_fastcdr_LIBRARY
      NAMES fastcdr
      PATHS 
        ${_ROS_ROOT}
        /usr 
        /usr/local 
      PATH_SUFFIXES lib
    )
  endif()

  # Find FastCDR include directory (for fastcdr/config.h)
  find_path(FastCDR_INCLUDE_DIR
    NAMES fastcdr/config.h
    PATHS 
      ${_ROS_ROOT}
      /usr 
      /usr/local 
    PATH_SUFFIXES include
  )

  # Also check alternative path pattern (fastcdr/fastcdr/config.h)
  if(NOT FastCDR_INCLUDE_DIR AND _ROS_ROOT)
    find_path(FastCDR_INCLUDE_DIR
      NAMES fastcdr/fastcdr/config.h
      PATHS ${_ROS_ROOT}
      PATH_SUFFIXES include
    )
  endif()

  if (FastDDS_INCLUDE_DIR AND FastDDS_fastdds_LIBRARY AND FastDDS_fastcdr_LIBRARY)
    set(FastDDS_FOUND TRUE)
    # Combine include directories, ensure both fastdds and fastcdr paths are included
    set(FastDDS_INCLUDE_DIRS ${FastDDS_INCLUDE_DIR})
    # Add fastcdr directory for fastcdr/config.h includes
    # In ROS, fastcdr headers are at fastcdr/fastcdr/config.h, but code uses fastcdr/config.h
    if(FastCDR_INCLUDE_DIR AND EXISTS "${FastCDR_INCLUDE_DIR}/fastcdr")
      # Add fastcdr directory so fastcdr/config.h resolves to fastcdr/fastcdr/config.h
      list(APPEND FastDDS_INCLUDE_DIRS "${FastCDR_INCLUDE_DIR}/fastcdr")
    endif()
    # In ROS, FastDDS headers are at fastrtps/fastdds/..., but code uses fastdds/...
    # Add fastrtps directory to include path so fastdds/... resolves to fastrtps/fastdds/...
    if(_ROS_ROOT AND EXISTS "${_ROS_ROOT}/include/fastrtps")
      list(APPEND FastDDS_INCLUDE_DIRS "${_ROS_ROOT}/include/fastrtps")
    endif()
    # Remove duplicates
    list(REMOVE_DUPLICATES FastDDS_INCLUDE_DIRS)
    set(FastDDS_LIBRARIES ${FastDDS_fastdds_LIBRARY} ${FastDDS_fastcdr_LIBRARY})
  else()
    set(FastDDS_FOUND FALSE)
  endif()
endif()

find_package_handle_standard_args(FastDDS DEFAULT_MSG FastDDS_FOUND)

mark_as_advanced(FastDDS_INCLUDE_DIR FastCDR_INCLUDE_DIR FastDDS_fastdds_LIBRARY FastDDS_fastcdr_LIBRARY)



