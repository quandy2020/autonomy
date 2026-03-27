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

# FindFoxglove.cmake
# This module looks for the Foxglove WebSocket C++ SDK.
#
# Once done, it defines:
#   foxglove_FOUND - system has foxglove
#   foxglove_INCLUDE_DIRS - foxglove include directories
#   foxglove_LIBRARIES - foxglove libraries
#
# It also defines imported targets for convenience:
#   foxglove::foxglove   (primary)
#   foxglove_sdk         (compat alias)

find_path(FOXGLOVE_INCLUDE_DIR
  NAMES foxglove/server.hpp
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

find_library(FOXGLOVE_LIBRARY
  NAMES foxglove
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /usr/local/lib
    /usr/lib
    ${CMAKE_PREFIX_PATH}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(foxglove DEFAULT_MSG FOXGLOVE_INCLUDE_DIR FOXGLOVE_LIBRARY)

if(foxglove_FOUND)
  set(foxglove_INCLUDE_DIRS ${FOXGLOVE_INCLUDE_DIR})
  set(foxglove_LIBRARIES ${FOXGLOVE_LIBRARY})

  if(NOT TARGET foxglove::foxglove)
    add_library(foxglove::foxglove UNKNOWN IMPORTED)
    set_target_properties(foxglove::foxglove PROPERTIES
      IMPORTED_LOCATION "${FOXGLOVE_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${FOXGLOVE_INCLUDE_DIR}"
    )
  endif()

  # Compatibility alias sometimes used by external projects.
  if(NOT TARGET foxglove_sdk)
    add_library(foxglove_sdk INTERFACE IMPORTED)
    set_target_properties(foxglove_sdk PROPERTIES
      INTERFACE_LINK_LIBRARIES "foxglove::foxglove"
    )
  endif()
endif()

mark_as_advanced(FOXGLOVE_INCLUDE_DIR FOXGLOVE_LIBRARY)
