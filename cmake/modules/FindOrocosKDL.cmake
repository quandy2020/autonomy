# Copyright 2026 The Openbot Authors
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

# @file FindOrocosKDL.cmake
# @brief Find-module for Orocos KDL (optional FK/IK for manipulation).
#
# @var OrocosKDL_FOUND
# @var OrocosKDL_INCLUDE_DIRS
# @var OrocosKDL_LIBRARIES
# @creates OrocosKDL::orocos-kdl

# Prefer the upstream CMake package (orocos_kdl-config.cmake).
find_package(orocos_kdl QUIET CONFIG
  PATHS
    /opt/homebrew
    /usr/local
    ${CMAKE_INSTALL_PREFIX}
)

if(orocos_kdl_FOUND)
  set(OrocosKDL_FOUND TRUE)
  set(OrocosKDL_INCLUDE_DIRS ${orocos_kdl_INCLUDE_DIRS})
  set(OrocosKDL_LIBRARIES ${orocos_kdl_LIBRARIES})
  if(TARGET orocos-kdl AND NOT TARGET OrocosKDL::orocos-kdl)
    add_library(OrocosKDL::orocos-kdl ALIAS orocos-kdl)
  endif()
else()
  find_path(OrocosKDL_INCLUDE_DIR
    NAMES kdl/chain.hpp
    PATHS
      ${CMAKE_INSTALL_PREFIX}/include
      /opt/homebrew/include
      /usr/local/include
      /usr/include
  )
  find_library(OrocosKDL_LIBRARY
    NAMES orocos-kdl
    PATHS
      ${CMAKE_INSTALL_PREFIX}/lib
      /opt/homebrew/lib
      /usr/local/lib
      /usr/lib
      /usr/lib/x86_64-linux-gnu
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(OrocosKDL DEFAULT_MSG
    OrocosKDL_LIBRARY OrocosKDL_INCLUDE_DIR)

  if(OrocosKDL_FOUND)
    set(OrocosKDL_LIBRARIES ${OrocosKDL_LIBRARY})
    set(OrocosKDL_INCLUDE_DIRS ${OrocosKDL_INCLUDE_DIR})
    if(NOT TARGET OrocosKDL::orocos-kdl)
      add_library(OrocosKDL::orocos-kdl UNKNOWN IMPORTED)
      set_target_properties(OrocosKDL::orocos-kdl PROPERTIES
        IMPORTED_LOCATION "${OrocosKDL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OrocosKDL_INCLUDE_DIR}")
    endif()
  endif()

  mark_as_advanced(OrocosKDL_INCLUDE_DIR OrocosKDL_LIBRARY)
endif()
