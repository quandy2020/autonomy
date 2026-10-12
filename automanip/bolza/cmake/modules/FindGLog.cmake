#=============================================================================
# Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# FindGLog.cmake — Google glog for bolza examples.
#
# Imported target: GLog::GLog
# Variables: GLog_FOUND, GLog_INCLUDE_DIRS, GLog_LIBRARIES
#
# A config package under /usr/local is preferred so the headers and the
# shared library are the same build. Mixing apt headers with that library
# fails at link time.
#=============================================================================
include(FindPackageHandleStandardArgs)

function(_bolza_glog_alias_existing)
  if(TARGET GLog::GLog)
    return()
  endif()
  if(NOT TARGET glog::glog)
    return()
  endif()
  get_target_property(_aliased glog::glog ALIASED_TARGET)
  if(_aliased)
    add_library(GLog::GLog ALIAS ${_aliased})
  else()
    add_library(GLog::GLog ALIAS glog::glog)
  endif()
endfunction()

if(TARGET GLog::GLog OR TARGET glog::glog)
  _bolza_glog_alias_existing()
  set(GLog_FOUND TRUE)
  set(GLog_LIBRARIES GLog::GLog)
  return()
endif()

foreach(_prefix IN ITEMS /usr/local /opt/homebrew "$ENV{HOME}/.local")
  if(EXISTS "${_prefix}/lib/cmake/glog/glog-config.cmake")
    set(glog_DIR "${_prefix}/lib/cmake/glog" CACHE PATH "Directory containing glog-config.cmake")
    break()
  endif()
endforeach()

find_package(glog CONFIG QUIET)
if(TARGET glog::glog)
  _bolza_glog_alias_existing()
  set(GLog_FOUND TRUE)
  set(GLog_LIBRARIES GLog::GLog)
  get_target_property(GLog_INCLUDE_DIRS glog::glog INTERFACE_INCLUDE_DIRECTORIES)
  message(STATUS "GLog: glog::glog (config)")
  return()
endif()

find_path(GLog_INCLUDE_DIRS
  NAMES glog/logging.h
  HINTS /usr/local/include /usr/include
)

find_library(GLog_LIBRARIES
  NAMES glog
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)

if(GLog_INCLUDE_DIRS MATCHES "/usr/local")
  find_library(GLog_LIBRARIES
    NAMES glog
    PATHS /usr/local
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
  )
endif()

find_package_handle_standard_args(GLog
  REQUIRED_VARS
    GLog_INCLUDE_DIRS
    GLog_LIBRARIES
)

mark_as_advanced(GLog_INCLUDE_DIRS GLog_LIBRARIES)

if(GLog_FOUND AND NOT TARGET GLog::GLog)
  add_library(GLog::GLog UNKNOWN IMPORTED)
  set_target_properties(GLog::GLog PROPERTIES
    IMPORTED_LOCATION "${GLog_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLog_INCLUDE_DIRS}"
  )
  message(STATUS "GLog: ${GLog_LIBRARIES} (includes ${GLog_INCLUDE_DIRS})")
endif()
