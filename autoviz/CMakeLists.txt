# Copyright 2026 The Openbot Authors (duyongquan)
#
# Autoviz — standalone Qt visualizer; depends on sibling autolink + automsgs only
# (does not build libautonomy).
#
# Standalone:
#   cmake -B build && cmake --build build --target autoviz
# Super-project (src/autonomy):
#   cmake -B build -DBUILD_AUTOVIZ=ON && cmake --build build --target autoviz
#
# cmake/ modules:
#   Config.cmake        — options, toolchain, Qt AUTOMOC
#   Dependencies.cmake  — embed autolink/automsgs, find Qt
#   Sources.cmake       — source glob and Ogre conditional sources
#   App.cmake           — autoviz executable and sample tools
#   OgreBackend.cmake   — optional Ogre render backend
#   Tests.cmake         — optional gtests (off by default)
#   Install.cmake       — install rules and Linux desktop integration

cmake_minimum_required(VERSION 3.20)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------------------
# Configure mode: standalone project vs Autonomy super-project subdirectory
# ---------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(autoviz LANGUAGES CXX C)
  set(AUTOVIZ_STANDALONE ON)
elseif(_AUTOVIZ_BUILDING_AS_SUBDIRECTORY)
  set(AUTOVIZ_STANDALONE OFF)
else()
  message(FATAL_ERROR
    "Configure from the autoviz directory (standalone), "
    "or enable -DBUILD_AUTOVIZ=ON from the Autonomy root.")
endif()

# Path conventions (used throughout):
#   AUTOVIZ_ROOT      — package root (resources/, autoviz/ source tree)
#   AUTOVIZ_DEPS_ROOT — sibling deps (autolink/, automsgs/, cmake/modules/)
#   AUTOVIZ_SRC_ROOT  — C++ sources (autoviz/autoviz/)
set(AUTOVIZ_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
get_filename_component(AUTOVIZ_DEPS_ROOT "${AUTOVIZ_ROOT}/.." ABSOLUTE)
set(AUTOVIZ_SRC_ROOT "${AUTOVIZ_ROOT}/autoviz")

include(Config)
include(Dependencies)
include(Sources)
include(App)

if(BUILD_AUTOVIZ_TESTS)
  include(Tests)
endif()

include(Install)

# Configure summary
if(AUTOVIZ_STANDALONE)
  set(_autoviz_mode "standalone")
else()
  set(_autoviz_mode "super-project")
endif()
message(STATUS "")
message(STATUS "Autoviz ${AUTOVIZ_VERSION} (${_autoviz_mode}): "
        "QML=${AUTOVIZ_USE_QML_VEHICLE} Ogre=${AUTOVIZ_USE_OGRE} tests=${BUILD_AUTOVIZ_TESTS}")
message(STATUS "")
