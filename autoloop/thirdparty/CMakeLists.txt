# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
#
# Git submodules: civetweb, foxglove/mcap.
# autolink / glog / protobuf / gtest / yaml-cpp: workspace or find_package.

# civetweb — HTTP helper; keep optional deps off.
set(CIVETWEB_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_SERVER_EXECUTABLE OFF CACHE BOOL "" FORCE)
set(CIVETWEB_INSTALL_EXECUTABLE OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_SSL OFF CACHE BOOL "" FORCE)
set(CIVETWEB_ENABLE_ASAN OFF CACHE BOOL "" FORCE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/civetweb/CMakeLists.txt")
  add_subdirectory(civetweb EXCLUDE_FROM_ALL)
  if(TARGET civetweb-c-library AND NOT TARGET autoloop_civetweb)
    add_library(autoloop_civetweb ALIAS civetweb-c-library)
  endif()
endif()

# mcap C++ SDK is header-only (cpp/mcap/include); see cmake/MCAP.cmake.
