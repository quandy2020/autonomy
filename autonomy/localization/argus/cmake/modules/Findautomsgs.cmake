# Copyright 2026 The Openbot Authors
#
# find_package(automsgs REQUIRED)
#   → automsgs::automsgs (or automsgs), AUTOMSGS_INCLUDE_DIRS, AUTOMSGS_FOUND
#
# Resolves: already-built CMake target → Config package → headers + lib.

include(FindPackageHandleStandardArgs)

macro(_argus_automsgs_set_found)
  set(AUTOMSGS_FOUND TRUE)
  set(automsgs_FOUND TRUE)
  if(TARGET automsgs::automsgs)
    set(AUTOMSGS_LIBRARIES automsgs::automsgs)
  elseif(TARGET automsgs)
    set(AUTOMSGS_LIBRARIES automsgs)
    if(NOT TARGET automsgs::automsgs)
      add_library(automsgs::automsgs ALIAS automsgs)
    endif()
  endif()
endmacro()

if(TARGET automsgs::automsgs OR TARGET automsgs)
  _argus_automsgs_set_found()
  find_package_handle_standard_args(automsgs DEFAULT_MSG AUTOMSGS_LIBRARIES)
  return()
endif()

# Config / prefix hints (install tree + monorepo sibling).
# Monorepo: <root>/automsgs alongside <root>/autonomy/localization/argus.
set(_automsgs_prefix_hints
  "$ENV{AUTOMSGS_ROOT}"
  "$ENV{AUTOMSGS_DIR}"
  "${CMAKE_SOURCE_DIR}/automsgs"
  "${CMAKE_SOURCE_DIR}/../automsgs"
  "${PROJECT_SOURCE_DIR}/../../../automsgs"
  "${PROJECT_SOURCE_DIR}/../../../../automsgs"
  /opt/homebrew
  /usr/local
  "$ENV{HOME}/.local"
)

foreach(_prefix IN LISTS _automsgs_prefix_hints)
  if(NOT _prefix)
    continue()
  endif()
  foreach(_cfg IN ITEMS
      "${_prefix}/lib/cmake/automsgs/automsgsConfig.cmake"
      "${_prefix}/lib/cmake/automsgs/automsgs-config.cmake"
      "${_prefix}/share/automsgs/cmake/automsgsConfig.cmake")
    if(EXISTS "${_cfg}")
      get_filename_component(automsgs_DIR "${_cfg}" DIRECTORY CACHE)
      break()
    endif()
  endforeach()
  if(automsgs_DIR)
    break()
  endif()
endforeach()

find_package(automsgs QUIET NO_MODULE)
if(TARGET automsgs::automsgs OR TARGET automsgs)
  _argus_automsgs_set_found()
  find_package_handle_standard_args(automsgs DEFAULT_MSG AUTOMSGS_LIBRARIES)
  mark_as_advanced(automsgs_DIR)
  return()
endif()

find_path(AUTOMSGS_INCLUDE_DIR
  NAMES automsgs/msgs/sensor_msgs/imu.pb.h
        automsgs/msgs/sensor_msgs/point_cloud2.pb.h
  HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
        $ENV{AUTOMSGS_ROOT} $ENV{AUTOMSGS_DIR}
  PATHS ${_automsgs_prefix_hints}
  PATH_SUFFIXES include
)

find_library(AUTOMSGS_LIBRARY
  NAMES automsgs
  HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
        $ENV{AUTOMSGS_ROOT} $ENV{AUTOMSGS_DIR}
  PATHS ${_automsgs_prefix_hints}
  PATH_SUFFIXES lib lib64
)

if(AUTOMSGS_INCLUDE_DIR AND AUTOMSGS_LIBRARY)
  set(AUTOMSGS_INCLUDE_DIRS "${AUTOMSGS_INCLUDE_DIR}")
  set(AUTOMSGS_LIBRARIES "${AUTOMSGS_LIBRARY}")
  if(NOT TARGET automsgs::automsgs)
    add_library(automsgs::automsgs UNKNOWN IMPORTED)
    set_target_properties(automsgs::automsgs PROPERTIES
      IMPORTED_LOCATION "${AUTOMSGS_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${AUTOMSGS_INCLUDE_DIR}")
  endif()
  _argus_automsgs_set_found()
endif()

find_package_handle_standard_args(automsgs
  REQUIRED_VARS AUTOMSGS_LIBRARIES AUTOMSGS_INCLUDE_DIR)

mark_as_advanced(automsgs_DIR AUTOMSGS_INCLUDE_DIR AUTOMSGS_LIBRARY)
unset(_automsgs_prefix_hints)
unset(_prefix)
unset(_cfg)
