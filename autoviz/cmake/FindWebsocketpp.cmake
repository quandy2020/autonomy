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

# FindWebsocketpp.cmake
# This module looks for websocketpp headers.
# Once done, it defines:
#   WEBSOCKETPP_FOUND - system has websocketpp
#   WEBSOCKETPP_INCLUDE_DIRS - websocketpp include directories

find_path(WEBSOCKETPP_INCLUDE_DIR
  NAMES websocketpp/version.hpp
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Websocketpp DEFAULT_MSG WEBSOCKETPP_INCLUDE_DIR)

if(WEBSOCKETPP_FOUND)
  set(WEBSOCKETPP_INCLUDE_DIRS ${WEBSOCKETPP_INCLUDE_DIR})

  # Keep compatibility with projects that link websocketpp::websocketpp.
  # Guard creation to avoid duplicate imported target definitions.
  if(NOT TARGET websocketpp::websocketpp)
    add_library(websocketpp::websocketpp INTERFACE IMPORTED)
    set_target_properties(websocketpp::websocketpp PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${WEBSOCKETPP_INCLUDE_DIRS}"
    )
  endif()
endif()

mark_as_advanced(WEBSOCKETPP_INCLUDE_DIR)
