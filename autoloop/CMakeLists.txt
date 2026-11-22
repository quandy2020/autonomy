# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License").

cmake_minimum_required(VERSION 3.20)
project(autoloop VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AUTOLOOP_SRC_ROOT "${CMAKE_SOURCE_DIR}/autoloop" CACHE PATH
  "Source root for common/ robot/ cloud/ includes")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(AutoloopHelpers)
include(Versioning)
include(SafetyChecks)
include(Proto)
include(Glog)
include(MCAP)
include(AutoLink)
include(WorkspaceDeps)

option(BUILD_TESTING "Build Autoloop unit tests" ON)
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest QUIET)
endif()

add_subdirectory(thirdparty)
add_subdirectory(autoloop/common)
add_subdirectory(autoloop/robot)
add_subdirectory(autoloop/cloud)
