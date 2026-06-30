# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# FindFBow.cmake
#
# Locates the stella-cv FBoW (Fast Bag of Words) library used by atlas.
# Resolution order:
#   1. CMake package config (fbowConfig.cmake from a system install)
#   2. Header/library search on the system
#   3. Bundled sources under autonomy/localization/stella_vslam/3rd/FBoW
#
# Defines:
#   FBow_FOUND
#   FBow_INCLUDE_DIRS
#   FBow_LIBRARIES
#   FBow::fbow          - imported or built target
#
# Legacy aliases (stella_vslam compatibility):
#   fbow_FOUND, fbow_INCLUDE_DIRS, fbow_LIBRARIES, fbow_LIBS

if(FBow_FOUND)
  return()
endif()

if(TARGET FBow::fbow OR TARGET fbow::fbow)
  set(FBow_FOUND TRUE)
  if(TARGET fbow::fbow AND NOT TARGET FBow::fbow)
    add_library(FBow::fbow ALIAS fbow::fbow)
  elseif(TARGET FBow::fbow AND NOT TARGET fbow::fbow)
    add_library(fbow::fbow ALIAS FBow::fbow)
  endif()
  return()
endif()

# ---------------------------------------------------------------------------
# 1. Installed CMake package (share/cmake/fbow/fbowConfig.cmake)
# ---------------------------------------------------------------------------

find_package(
  fbow
  CONFIG
  QUIET
  PATHS
    ${CMAKE_PREFIX_PATH}
    $ENV{FBOW_DIR}
    /usr/local
    /usr
  PATH_SUFFIXES
    lib/cmake/fbow
    share/cmake/fbow
)

if(fbow_FOUND OR TARGET fbow::fbow)
  set(FBow_FOUND TRUE)
  if(DEFINED fbow_INCLUDE_DIRS)
    set(FBow_INCLUDE_DIRS ${fbow_INCLUDE_DIRS})
  endif()
  if(TARGET fbow::fbow)
    set(FBow_LIBRARIES fbow::fbow)
  elseif(DEFINED fbow_LIBS)
    set(FBow_LIBRARIES ${fbow_LIBS})
  else()
    set(FBow_LIBRARIES fbow)
  endif()
endif()

# ---------------------------------------------------------------------------
# 2. Manual header/library search
# ---------------------------------------------------------------------------

if(NOT FBow_FOUND)
  find_path(
    FBOW_INCLUDE_DIR
    NAMES fbow/vocabulary.h
    HINTS
      ENV FBOW_DIR
    PATH_SUFFIXES include
    PATHS
      /usr/local
      /usr
      ${CMAKE_INSTALL_PREFIX}
  )

  find_library(
    FBOW_LIBRARY
    NAMES fbow
    HINTS
      ENV FBOW_DIR
    PATH_SUFFIXES lib lib64
    PATHS
      /usr/local
      /usr
      ${CMAKE_INSTALL_PREFIX}
  )

  if(FBOW_INCLUDE_DIR AND FBOW_LIBRARY)
    set(FBow_FOUND TRUE)
    set(FBow_INCLUDE_DIRS ${FBOW_INCLUDE_DIR})
    set(FBow_LIBRARIES ${FBOW_LIBRARY})

    if(NOT TARGET FBow::fbow)
      add_library(FBow::fbow UNKNOWN IMPORTED)
      set_target_properties(
        FBow::fbow
        PROPERTIES
          IMPORTED_LOCATION "${FBOW_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${FBOW_INCLUDE_DIR}"
      )
      add_library(fbow::fbow ALIAS FBow::fbow)
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# 3. Bundled stella-cv FBoW under stella_vslam/3rd
# ---------------------------------------------------------------------------

if(NOT FBow_FOUND)
  if(NOT FBOW_ROOT)
    set(
      FBOW_ROOT
      "${CMAKE_CURRENT_LIST_DIR}/../../autonomy/localization/stella_vslam/3rd/FBoW"
      CACHE PATH "Path to stella-cv FBoW sources"
    )
  endif()

  if(EXISTS "${FBOW_ROOT}/CMakeLists.txt")
    set(FBOW_BUILD_TESTS OFF CACHE BOOL "Build bundled FBoW tests" FORCE)
    set(FBOW_BUILD_UTILS OFF CACHE BOOL "Build bundled FBoW utils" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "Build bundled FBoW tests" FORCE)
    set(BUILD_UTILS OFF CACHE BOOL "Build bundled FBoW utils" FORCE)

    if(NOT TARGET fbow)
      add_subdirectory(
        "${FBOW_ROOT}"
        "${CMAKE_BINARY_DIR}/thirdparty/fbow"
        EXCLUDE_FROM_ALL
      )
    endif()

    if(TARGET fbow)
      set(FBow_FOUND TRUE)
      set(FBow_INCLUDE_DIRS "${FBOW_ROOT}/include")
      set(FBow_LIBRARIES fbow)

      if(NOT TARGET FBow::fbow)
        add_library(FBow::fbow ALIAS fbow)
      endif()
      if(NOT TARGET fbow::fbow)
        add_library(fbow::fbow ALIAS fbow)
      endif()
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# Ensure FBow::fbow target exists for target_link_libraries()
# ---------------------------------------------------------------------------

if(FBow_FOUND AND NOT TARGET FBow::fbow)
  if(TARGET fbow::fbow)
    add_library(FBow::fbow ALIAS fbow::fbow)
  elseif(TARGET fbow)
    add_library(FBow::fbow ALIAS fbow)
  elseif(FBOW_LIBRARY AND FBOW_INCLUDE_DIR)
    add_library(FBow::fbow UNKNOWN IMPORTED)
    set_target_properties(
      FBow::fbow
      PROPERTIES
        IMPORTED_LOCATION "${FBOW_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${FBOW_INCLUDE_DIR}"
    )
  elseif(FBow_LIBRARIES AND FBow_INCLUDE_DIRS)
    add_library(FBow::fbow UNKNOWN IMPORTED)
    set_target_properties(
      FBow::fbow
      PROPERTIES
        IMPORTED_LOCATION "${FBow_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${FBow_INCLUDE_DIRS}"
    )
  endif()
endif()

if(FBow_FOUND AND NOT TARGET fbow::fbow)
  if(TARGET FBow::fbow)
    add_library(fbow::fbow ALIAS FBow::fbow)
  elseif(TARGET fbow)
    add_library(fbow::fbow ALIAS fbow)
  endif()
endif()

# ---------------------------------------------------------------------------
# Legacy variable names used by stella_vslam / atlas
# ---------------------------------------------------------------------------

if(FBow_FOUND)
  set(fbow_FOUND TRUE)
  set(fbow_INCLUDE_DIRS ${FBow_INCLUDE_DIRS})
  set(fbow_LIBRARIES ${FBow_LIBRARIES})
  set(fbow_LIBS ${FBow_LIBRARIES})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  FBow
  DEFAULT_MSG
  FBow_LIBRARIES
  FBow_INCLUDE_DIRS
)

mark_as_advanced(FBOW_INCLUDE_DIR FBOW_LIBRARY FBOW_ROOT)
