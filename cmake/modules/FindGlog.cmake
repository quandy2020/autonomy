# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
#
# Locate the GLOG library (Linux /usr/local and macOS Homebrew).

find_path(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  PATHS
    /opt/homebrew/include
    /usr/local/include
    /usr/include
)

find_library(GLOG_LIBRARY
  NAMES glog
  PATHS
    /opt/homebrew/lib
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
)

if(GLOG_INCLUDE_DIR AND GLOG_LIBRARY)
  set(GLOG_FOUND TRUE)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
else()
  set(GLOG_FOUND FALSE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)

if(GLOG_FOUND AND NOT TARGET glog::glog)
  add_library(glog::glog UNKNOWN IMPORTED)
  set_target_properties(glog::glog PROPERTIES
    IMPORTED_LOCATION "${GLOG_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}")
  # Alias bare name used by some targets (prefer full path via imported target).
  if(NOT TARGET glog)
    add_library(glog ALIAS glog::glog)
  endif()
endif()

mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)
