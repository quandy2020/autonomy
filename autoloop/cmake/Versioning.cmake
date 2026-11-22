# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

set(AUTOLOOP_VERSION "0.1.0")
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE AUTOLOOP_GIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT AUTOLOOP_GIT_SHA)
  set(AUTOLOOP_GIT_SHA "unknown")
endif()
add_compile_definitions(
  AUTOLOOP_VERSION="${AUTOLOOP_VERSION}"
  AUTOLOOP_GIT_SHA="${AUTOLOOP_GIT_SHA}")
