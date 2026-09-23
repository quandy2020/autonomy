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

# @brief find_package wrapper that defers REQUIRED failures.
#
# Missing REQUIRED packages are collected and reported by
# autocmake_build instead of aborting at the first failure.
# Packages that are not PRIVATE are written into the exported
# package config as find_dependency().
#
# @param package_name Package name passed to find_package.
# @param REQUIRED Fail the configure step when the package is missing.
# @param PRIVATE Do not re-export this dependency.
# @param QUIET Forward QUIET to find_package.
# @param CONFIG Forward CONFIG to find_package.
# @param EXACT Forward EXACT to find_package.
# @param VERSION Version passed to find_package.
# @param COMPONENTS Components passed to find_package.
# @param EXTRA_ARGUMENTS Extra arguments forwarded to find_package.
macro(autocmake_find package_name)
  _autocmake_parse(_autocmake_fp
    "REQUIRED;PRIVATE;QUIET;CONFIG;EXACT"
    "VERSION"
    "COMPONENTS;EXTRA_ARGUMENTS"
    ${ARGN})

  set(_autocmake_fp_args "${package_name}")
  if(_autocmake_fp_VERSION)
    list(APPEND _autocmake_fp_args "${_autocmake_fp_VERSION}")
  endif()
  if(_autocmake_fp_EXACT)
    list(APPEND _autocmake_fp_args EXACT)
  endif()
  if(_autocmake_fp_QUIET)
    list(APPEND _autocmake_fp_args QUIET)
  endif()
  if(_autocmake_fp_CONFIG)
    list(APPEND _autocmake_fp_args CONFIG)
  endif()
  if(_autocmake_fp_COMPONENTS)
    list(APPEND _autocmake_fp_args COMPONENTS ${_autocmake_fp_COMPONENTS})
  endif()
  if(_autocmake_fp_EXTRA_ARGUMENTS)
    list(APPEND _autocmake_fp_args ${_autocmake_fp_EXTRA_ARGUMENTS})
  endif()

  find_package(${_autocmake_fp_args})

  if(${package_name}_FOUND)
    if(NOT _autocmake_fp_PRIVATE)
      set(_autocmake_dep "find_dependency(${package_name}")
      if(_autocmake_fp_VERSION)
        string(APPEND _autocmake_dep " ${_autocmake_fp_VERSION}")
      endif()
      if(_autocmake_fp_EXACT)
        string(APPEND _autocmake_dep " EXACT")
      endif()
      if(_autocmake_fp_COMPONENTS)
        string(JOIN " " _autocmake_fp_components ${_autocmake_fp_COMPONENTS})
        string(APPEND _autocmake_dep " COMPONENTS ${_autocmake_fp_components}")
      endif()
      string(APPEND _autocmake_dep ")")
      set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES "${_autocmake_dep}")
    endif()
  elseif(_autocmake_fp_REQUIRED)
    autocmake_error("${package_name} was requested and not found")
  elseif(NOT _autocmake_fp_QUIET)
    message(STATUS "autocmake: optional package ${package_name} was not found")
  endif()
endmacro()

# @brief Link packages that have already been found.
#
# Prefers ${Package}::${Package}, then a target of the same name, then the
# classic ${Package}_LIBRARIES / ${Package}_INCLUDE_DIRS variables.
#
# @param target Existing target.
# @param PRIVATE Link PRIVATE instead of PUBLIC.
# @param INTERFACE Link INTERFACE instead of PUBLIC.
# @param ARGN Package names.
function(autocmake_link target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "autocmake_link: '${target}' is not a target")
  endif()
  _autocmake_parse(_arg "PRIVATE;INTERFACE" "" "" ${ARGN})
  if(_arg_INTERFACE)
    set(_scope INTERFACE)
  elseif(_arg_PRIVATE)
    set(_scope PRIVATE)
  else()
    set(_scope PUBLIC)
  endif()

  foreach(_pkg IN LISTS _arg_UNPARSED_ARGUMENTS)
    if(TARGET ${_pkg}::${_pkg})
      target_link_libraries(${target} ${_scope} ${_pkg}::${_pkg})
    elseif(TARGET ${_pkg})
      target_link_libraries(${target} ${_scope} ${_pkg})
    elseif(DEFINED ${_pkg}_LIBRARIES OR DEFINED ${_pkg}_INCLUDE_DIRS)
      if(${_pkg}_INCLUDE_DIRS)
        target_include_directories(${target} ${_scope} ${${_pkg}_INCLUDE_DIRS})
      endif()
      if(${_pkg}_LIBRARIES)
        target_link_libraries(${target} ${_scope} ${${_pkg}_LIBRARIES})
      endif()
    else()
      message(FATAL_ERROR
        "autocmake_link: '${_pkg}' has no target or _LIBRARIES")
    endif()
  endforeach()
endfunction()
