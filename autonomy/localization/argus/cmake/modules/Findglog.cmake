# Copyright 2026 The Openbot Authors
# Adapted from autolink FindGlog (Apache-2.0).
#
# find_package(glog REQUIRED)
#   → glog::glog, GLOG_INCLUDE_DIR, GLOG_LIBRARY, GLOG_FOUND
#
# Prefers a matching include/lib pair (Homebrew /usr/local before apt) so
# glog 0.6+ headers are not mixed with system libglog.so.0.

include(FindPackageHandleStandardArgs)

if(TARGET glog::glog)
  set(GLOG_FOUND TRUE)
  set(glog_FOUND TRUE)
  set(GLOG_LIBRARIES glog::glog)
  find_package_handle_standard_args(glog DEFAULT_MSG GLOG_LIBRARIES)
  return()
endif()

# Prefer Config packages under common prefixes (ABI-safe).
foreach(_glog_cfg_prefix IN ITEMS
    "/opt/homebrew"
    "/usr/local"
    "$ENV{HOME}/.local")
  if(EXISTS "${_glog_cfg_prefix}/lib/cmake/glog/glog-config.cmake")
    set(glog_DIR "${_glog_cfg_prefix}/lib/cmake/glog" CACHE PATH
      "Directory containing glog-config.cmake" FORCE)
    break()
  endif()
  if(EXISTS "${_glog_cfg_prefix}/lib64/cmake/glog/glog-config.cmake")
    set(glog_DIR "${_glog_cfg_prefix}/lib64/cmake/glog" CACHE PATH
      "Directory containing glog-config.cmake" FORCE)
    break()
  endif()
endforeach()

find_package(glog QUIET NO_MODULE)
if(TARGET glog::glog)
  get_target_property(_glog_inc glog::glog INTERFACE_INCLUDE_DIRECTORIES)
  if(_glog_inc)
    set(GLOG_INCLUDE_DIR "${_glog_inc}")
    set(GLOG_INCLUDE_DIRS "${_glog_inc}")
  endif()
  get_target_property(_glog_loc glog::glog IMPORTED_LOCATION_RELEASE)
  if(NOT _glog_loc)
    get_target_property(_glog_loc glog::glog IMPORTED_LOCATION)
  endif()
  if(_glog_loc)
    set(GLOG_LIBRARY "${_glog_loc}" CACHE FILEPATH "glog library" FORCE)
  endif()
  set(GLOG_LIBRARIES glog::glog)
  set(GLOG_FOUND TRUE)
  set(glog_FOUND TRUE)
  find_package_handle_standard_args(glog DEFAULT_MSG GLOG_LIBRARIES)
  mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)
  return()
endif()

# Module fallback: Homebrew / local before system multiarch.
set(_glog_search_paths
  "$ENV{HOME}/.local"
  /opt/homebrew
  /usr/local
)

find_path(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
  PATHS ${_glog_search_paths}
  PATH_SUFFIXES include
)

find_library(GLOG_LIBRARY
  NAMES glog
  HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
  PATHS ${_glog_search_paths}
  PATH_SUFFIXES lib lib64
)

if(GLOG_INCLUDE_DIR MATCHES "/usr/local")
  find_library(GLOG_LIBRARY
    NAMES glog
    PATHS /usr/local
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
    NO_CMAKE_SYSTEM_PATH)
elseif(GLOG_INCLUDE_DIR MATCHES "/opt/homebrew")
  find_library(GLOG_LIBRARY
    NAMES glog
    PATHS /opt/homebrew
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
    NO_CMAKE_SYSTEM_PATH)
endif()

if(NOT GLOG_INCLUDE_DIR OR NOT GLOG_LIBRARY)
  find_path(GLOG_INCLUDE_DIR NAMES glog/logging.h)
  find_library(GLOG_LIBRARY NAMES glog)
endif()

if(GLOG_INCLUDE_DIR AND GLOG_LIBRARY)
  set(GLOG_FOUND TRUE)
  set(glog_FOUND TRUE)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  if(NOT TARGET glog::glog)
    add_library(glog::glog UNKNOWN IMPORTED)
    set_target_properties(glog::glog PROPERTIES
      IMPORTED_LOCATION "${GLOG_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}")
  endif()
else()
  set(GLOG_FOUND FALSE)
  set(glog_FOUND FALSE)
endif()

# Legacy un-namespaced target some packages export.
if(TARGET glog AND NOT TARGET glog::glog)
  add_library(glog::glog ALIAS glog)
  set(GLOG_FOUND TRUE)
  set(glog_FOUND TRUE)
  set(GLOG_LIBRARIES glog::glog)
endif()

find_package_handle_standard_args(glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)
mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)
unset(_glog_search_paths)
unset(_glog_inc)
unset(_glog_loc)
unset(_glog_cfg_prefix)
