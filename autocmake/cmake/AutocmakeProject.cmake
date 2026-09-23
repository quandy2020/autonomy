# Copyright 2026 The Openbot Authors (duyongquan)
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

include_guard(GLOBAL)

# @brief Set install layout, C++ defaults, and the export set name.
#
# Call from the top-level CMakeLists after project(name VERSION x.y.z).
#
# @param INCLUDE Public include prefix. Default: PROJECT_NAME.
# @param SUFFIX Prerelease tag, recorded only (not part of the CMake package version).
# @param EXTRAS Extra .cmake or .cmake.in files installed next to the package config.
macro(autocmake_project)
  _autocmake_parse(autocmake_project "" "INCLUDE;SUFFIX" "EXTRAS" ${ARGN})

  if(NOT PROJECT_NAME OR PROJECT_NAME STREQUAL "Project")
    message(FATAL_ERROR "autocmake_project: call project() first")
  endif()
  _autocmake_ensure_xml()
  if(NOT PROJECT_VERSION AND _autocmake_xml_done)
    set(PROJECT_VERSION "${${PROJECT_NAME}_VERSION}")
  endif()
  if(NOT PROJECT_VERSION)
    message(FATAL_ERROR "autocmake_project: set project(VERSION) or provide package.xml")
  endif()
  if(NOT PROJECT_VERSION_MAJOR)
    string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$" _autocmake_ver "${PROJECT_VERSION}")
    if(NOT _autocmake_ver)
      message(FATAL_ERROR
        "autocmake_project: version '${PROJECT_VERSION}' is not MAJOR.MINOR.PATCH")
    endif()
    set(PROJECT_VERSION_MAJOR "${CMAKE_MATCH_1}")
    set(PROJECT_VERSION_MINOR "${CMAKE_MATCH_2}")
    set(PROJECT_VERSION_PATCH "${CMAKE_MATCH_3}")
  endif()

  include(GNUInstallDirs)

  set(PROJECT_NAME_LOWER "${PROJECT_NAME}")
  string(TOLOWER "${PROJECT_NAME_LOWER}" PROJECT_NAME_LOWER)
  string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
  set(PROJECT_LIBRARY_TARGET_NAME "${PROJECT_NAME}")
  set(PROJECT_EXPORT_NAME "${PROJECT_NAME}")
  set(AUTOCMAKE_EXPORT_NAME "${PROJECT_NAME}Targets")

  if(autocmake_project_INCLUDE)
    set(PROJECT_INCLUDE_DIR "${autocmake_project_INCLUDE}")
  else()
    set(PROJECT_INCLUDE_DIR "${PROJECT_NAME}")
  endif()

  set(PROJECT_VERSION_FULL "${PROJECT_VERSION}")
  if(autocmake_project_SUFFIX)
    set(PROJECT_VERSION_SUFFIX "${autocmake_project_SUFFIX}")
    set(PROJECT_VERSION_FULL "${PROJECT_VERSION}~${PROJECT_VERSION_SUFFIX}")
  endif()

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  set(AUTOCMAKE_LIB_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}")
  set(AUTOCMAKE_BIN_INSTALL_DIR "${CMAKE_INSTALL_BINDIR}")
  set(AUTOCMAKE_INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
  set(AUTOCMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_BUILD_ERRORS "")
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_BUILD_WARNINGS "")
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES "")
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_CONFIG_EXTRAS "")

  foreach(_extra IN LISTS autocmake_project_EXTRAS)
    if(NOT EXISTS "${_extra}")
      message(FATAL_ERROR "autocmake_project: EXTRAS file does not exist: ${_extra}")
    endif()
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_CONFIG_EXTRAS "${_extra}")
  endforeach()

  message(STATUS "${PROJECT_NAME} version ${PROJECT_VERSION_FULL}")
endmacro()

# @brief Turn on CTest and compiler warnings.
#
# Loads package.xml dependencies when autocmake_dependencies() has not run yet.
#
# @param QUIT Stop configuration when autocmake_find recorded a REQUIRED failure.
macro(autocmake_build)
  _autocmake_parse(autocmake_build "QUIT" "" "" ${ARGN})

  _autocmake_ensure_xml()
  get_property(_autocmake_dependencies_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_DEPENDENCIES_DONE)
  if(_autocmake_xml_done AND NOT _autocmake_dependencies_done)
    autocmake_dependencies()
  endif()
  _autocmake_options()

  get_property(_autocmake_warnings DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_BUILD_WARNINGS)
  if(_autocmake_warnings)
    message(WARNING "autocmake configuration warnings:\n  ${_autocmake_warnings}")
  endif()

  get_property(_autocmake_errors DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_BUILD_ERRORS)
  if(_autocmake_errors)
    if(autocmake_build_QUIT)
      message(FATAL_ERROR "autocmake configuration errors:\n  ${_autocmake_errors}")
    else()
      message(SEND_ERROR "autocmake configuration errors:\n  ${_autocmake_errors}")
    endif()
  endif()

  if(NOT _autocmake_errors)
  include(CTest)
  _autocmake_add_codecheck()
  message(STATUS "autocmake: build configuration successful")
  endif()
endmacro()
