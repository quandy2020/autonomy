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
# Prefer the CMake package from docker/install/install_osqp.sh when present,
# then fall back to header/library search under /usr/local and $HOME/.local
# (installer_base.sh prefixes). Autonomy sources include "osqp/osqp.h", so the
# include root must be .../include (not .../include/osqp from osqp::osqp).
#
# @var OSQP_FOUND         Whether OSQP was found
# @var OSQP_INCLUDE_DIRS  Include directories
# @var OSQP_LIBRARIES     Libraries to link
# @creates OSQP::OSQP     IMPORTED target (when found)

if(TARGET OSQP::OSQP)
  set(OSQP_FOUND TRUE)
  return()
endif()

set(_OSQP_HINT_INCLUDE
  ${CMAKE_INSTALL_PREFIX}/include
  $ENV{HOME}/.local/include
  /opt/homebrew/include
  /usr/local/include
  /usr/include
)
set(_OSQP_HINT_LIB
  ${CMAKE_INSTALL_PREFIX}/lib
  ${CMAKE_INSTALL_PREFIX}/lib64
  $ENV{HOME}/.local/lib
  $ENV{HOME}/.local/lib64
  /opt/homebrew/lib
  /usr/local/lib
  /usr/local/lib64
  /usr/lib
  /usr/lib64
  /usr/lib/x86_64-linux-gnu
  /usr/lib/aarch64-linux-gnu
)

# Header layout required by autonomy: osqp/osqp.h under include root.
find_path(OSQP_INCLUDE_DIR
  NAMES osqp/osqp.h
  HINTS ${_OSQP_HINT_INCLUDE}
  PATHS ${_OSQP_HINT_INCLUDE}
)

# 1) Shared library via official package (lib/cmake/osqp/).
find_package(osqp CONFIG QUIET)
if(NOT OSQP_LIBRARY AND osqp_FOUND AND TARGET osqp::osqp)
  get_target_property(_osqp_loc osqp::osqp IMPORTED_LOCATION_RELEASE)
  if(NOT _osqp_loc)
    get_target_property(_osqp_loc osqp::osqp IMPORTED_LOCATION)
  endif()
  if(NOT _osqp_loc)
    get_target_property(_osqp_configs osqp::osqp IMPORTED_CONFIGURATIONS)
    if(_osqp_configs)
      list(GET _osqp_configs 0 _osqp_cfg)
      get_target_property(_osqp_loc osqp::osqp IMPORTED_LOCATION_${_osqp_cfg})
    endif()
  endif()
  if(_osqp_loc)
    set(OSQP_LIBRARY "${_osqp_loc}" CACHE FILEPATH "OSQP library" FORCE)
  endif()
endif()

# 2) Manual library search.
if(NOT OSQP_LIBRARY)
  find_library(OSQP_LIBRARY
    NAMES osqp
    HINTS ${_OSQP_HINT_LIB}
    PATHS ${_OSQP_HINT_LIB}
  )
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OSQP
  REQUIRED_VARS OSQP_LIBRARY OSQP_INCLUDE_DIR
  FAIL_MESSAGE
    "OSQP not found (need libosqp + osqp/osqp.h). Install with:\n\
  bash docker/install/install_osqp.sh\n\
or:\n\
  python3 -m install_deps --resume-from install_osqp.sh\n\
Then ensure CMAKE_PREFIX_PATH covers the install prefix (/usr/local or $HOME/.local)."
)

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
unset(_OSQP_HINT_INCLUDE)
unset(_OSQP_HINT_LIB)
unset(_osqp_loc)
unset(_osqp_configs)
unset(_osqp_cfg)
