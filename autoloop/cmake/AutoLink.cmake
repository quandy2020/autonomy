# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

# Prefer the workspace autolink checkout (sibling of this project).
find_path(AUTOLINK_INCLUDE_DIR
  NAMES autolink/time/time.hpp autolink/common/file.hpp
  HINTS "${CMAKE_SOURCE_DIR}/../autolink"
  PATH_SUFFIXES .)
if(AUTOLINK_INCLUDE_DIR)
  set(AUTOLOOP_WITH_AUTOLINK ON)
  add_library(autoloop_autolink_headers INTERFACE)
  target_include_directories(autoloop_autolink_headers INTERFACE ${AUTOLINK_INCLUDE_DIR})
  add_compile_definitions(AUTOLOOP_WITH_AUTOLINK=1)
  message(STATUS "Autoloop: autolink headers at ${AUTOLINK_INCLUDE_DIR}")
else()
  set(AUTOLOOP_WITH_AUTOLINK OFF)
  message(STATUS "Autoloop: autolink not found; Time/path wrap std")
endif()
