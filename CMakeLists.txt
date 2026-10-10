# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -----------------------------------------------------------------------------
# Autonomy umbrella package
#
# This file is the single entry point for the whole autonomy tree. It does
# not own application logic; it wires domains, embeds siblings, and installs.
#
# Layout
#   autonomy/*     domain libraries (common, map, planning, …)
#   automsgs       always embedded  — message / proto package
#   autolink       always embedded  — runtime / channel framework
#   autodriver     always embedded  — sensor & chassis HAL (companion)
#   autoviz        optional         — visualization (BUILD_AUTOVIZ)
#
# Supporting files
#   cmake/Functions.cmake     modules, embed, version, domains, install
#   cmake/ProtoHelper.cmake   domain .proto → autonomy_proto
#   cmake/modules/            Find* and EnsureProtobuf319
#   package.xml               third-party build_depend list
#
# Build
#   ./scripts/build_autonomy_autocmake.sh full|select|up-to …
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.20)

# Normalize install(DESTINATION) paths (CMake 3.31+).
if(POLICY CMP0177)
  cmake_policy(SET CMP0177 NEW)
endif()
# Prefer <PKG>_ROOT over legacy ROOT vars (Eigen/FLANN via PCL).
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
# Use BoostConfig instead of the removed FindBoost module.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
# SuiteSparse/Ceres → FindBLAS probes optional nvpl; skip when absent.
set(CMAKE_DISABLE_FIND_PACKAGE_nvpl TRUE)

project(autonomy VERSION 0.2.0 LANGUAGES CXX C)

# --- autocmake ---------------------------------------------------------------
# Prefer a system / prefix install; otherwise load the vendored tree under
# ./autocmake so a fresh checkout configures without a prior install.
find_package(autocmake CONFIG QUIET)
if(NOT autocmake_FOUND)
  set(AUTOCMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/autocmake")
  include("${AUTOCMAKE_DIR}/Autocmake.cmake")
endif()

# --- options -----------------------------------------------------------------
# Package-level switches. Per-domain toggles (AUTONOMY_BUILD_<MOD>) are
# declared later by autonomy_modules().
option(BUILD_GRPC        "Build gRPC / bridge support"              ON)
option(BUILD_PROMETHEUS  "Build Prometheus monitoring (system)"     OFF)
option(BUILD_DOCS        "Build Sphinx documentation"               OFF)
option(BUILD_TEST        "Build unit tests"                         OFF)
option(BUILD_TOOLS       "Build tools"                              ON)
option(BUILD_AUTOVIZ     "Build autoviz"                            ON)

# --- project defaults --------------------------------------------------------
# autocmake_test() keys off BUILD_TESTING; keep it in sync with BUILD_TEST.
if(DEFINED BUILD_TEST AND NOT BUILD_TEST)
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

# Default to Release when the user did not pass -DCMAKE_BUILD_TYPE=.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep binaries / libs under the build tree in predictable places.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find* modules first, then umbrella helpers (Functions pulls in ProtoHelper).
list(PREPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")
include("${PROJECT_SOURCE_DIR}/cmake/Functions.cmake")

# --- siblings & domains ------------------------------------------------------
# autonomy_modules()
#   GLOB autonomy/*/CMakeLists.txt, creates AUTONOMY_BUILD_<MOD>
#   options, fills AUTONOMY_ENABLED_MODULES.
#
# autonomy_embed_siblings()
#   add_subdirectory(automsgs / autolink) and write stub *Config.cmake so
#   later find_package / package.xml resolution sees them as FOUND.
#   Must run before autocmake_build().
autonomy_modules()
autonomy_embed_siblings()

# --- autocmake project -------------------------------------------------------
# autocmake_project()  — package metadata from package.xml / version.
# autocmake_build()    — find every <build_depend> (Eigen3, Ceres, …);
#                        QUIT aborts configure on the first REQUIRED miss.
#
autocmake_project()
autocmake_build(QUIT)

# Distro yaml-cpp / glog often export bare targets; domains link namespaced.
if(TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
if(TARGET glog AND NOT TARGET glog::glog)
  add_library(glog::glog ALIAS glog)
endif()

# Vendored CLI11 (autolink/thirdparty) — header-only process CLI.
if(NOT TARGET autonomy::cli11)
  add_library(autonomy_cli11 INTERFACE)
  add_library(autonomy::cli11 ALIAS autonomy_cli11)
  target_include_directories(autonomy_cli11 INTERFACE
    "${PROJECT_SOURCE_DIR}/autolink/thirdparty/CLI11/include")
endif()

# Generate autonomy/common/version.cpp and config.hpp from templates.
autonomy_configure_version()

if(BUILD_DOCS)
  add_subdirectory(docs)
endif()

# --- domain graph ------------------------------------------------------------
# autonomy_add_proto()
#   Collect .proto under enabled domains → shared target autonomy_proto
#   (plain pb + optional gRPC for *_service.proto).
#
# autonomy_add_domains()
#   add_subdirectory each enabled domain, then wire INTERFACE target
#   autonomy / autonomy::autonomy that links proto + all domain libs.
autonomy_add_proto()
autonomy_add_domains()

# --- companions --------------------------------------------------------------
# Built after domains so they can link autonomy / autolink / automsgs.
# Same idea as autodriver adding chassis/ from its root CMakeLists.
add_subdirectory(autodriver)           # required sensor / chassis HAL
if(BUILD_AUTOVIZ)
  add_subdirectory(autoviz)            # optional visualization stack
endif()

# --- install -----------------------------------------------------------------
# Domain public headers (+ generated *.pb.h) and cmake/modules/.
autonomy_install_headers()

# Write AutonomyConfig.cmake / package index (autocmake export).
autocmake_package()
