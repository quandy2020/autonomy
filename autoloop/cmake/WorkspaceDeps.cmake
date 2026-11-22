# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

# Sibling workspace: autolink thirdparty headers, automsgs, glog.

set(_ws "${CMAKE_SOURCE_DIR}/..")

if(NOT TARGET autoloop_cli11)
  add_library(autoloop_cli11 INTERFACE)
  target_include_directories(autoloop_cli11 INTERFACE
    "${_ws}/autolink/thirdparty/CLI11/include")
endif()

if(NOT TARGET autoloop_json)
  add_library(autoloop_json INTERFACE)
  target_include_directories(autoloop_json INTERFACE
    "${_ws}/autolink/thirdparty/json/include")
endif()

if(NOT TARGET automsgs)
  find_package(automsgs QUIET)
endif()
if(TARGET automsgs)
  set(AUTOLOOP_WITH_AUTOMSGS ON)
  message(STATUS "Autoloop: automsgs target found")
else()
  set(AUTOLOOP_WITH_AUTOMSGS OFF)
  message(STATUS "Autoloop: automsgs not in this CMake graph; link it when building from the workspace")
endif()
