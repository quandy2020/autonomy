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

# @brief cmake_parse_arguments plus a warning for unknown keywords.
macro(_autocmake_parse prefix options one_value multi_value)
  cmake_parse_arguments(${prefix} "${options}" "${one_value}" "${multi_value}" ${ARGN})
endmacro()

# @brief Record a configuration error. autocmake_build prints them.
macro(autocmake_error)
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_BUILD_ERRORS "${ARGN}")
endmacro()

# @brief Record a configuration warning. autocmake_build prints them.
macro(autocmake_warning)
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_BUILD_WARNINGS "${ARGN}")
endmacro()

# @brief RPATH that finds libraries in CMAKE_INSTALL_LIBDIR after install.
function(_autocmake_runtime_path name)
  if(APPLE)
    set(_origin "@loader_path")
  else()
    string(CONCAT _origin "$" "ORIGIN")
  endif()
  get_target_property(_type ${name} TYPE)
  if(_type STREQUAL "EXECUTABLE")
    file(RELATIVE_PATH _relative
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
    if(_relative STREQUAL ".")
      set(_path "${_origin}")
    else()
      set(_path "${_origin}/${_relative}")
    endif()
  elseif(_type MATCHES "SHARED_LIBRARY|MODULE_LIBRARY")
    set(_path "${_origin}")
  else()
    set(_path "")
  endif()
  set(_autocmake_runtime_path "${_path}" PARENT_SCOPE)
endfunction()

# @brief Install a compiled or interface target into the project export set.
function(_autocmake_install_target name)
  if(NOT AUTOCMAKE_EXPORT_NAME)
    message(FATAL_ERROR
      "_autocmake_install_target: call autocmake_project() before adding targets")
  endif()
  _autocmake_runtime_path(${name})
  if(_autocmake_runtime_path)
    set_target_properties(${name} PROPERTIES INSTALL_RPATH "${_autocmake_runtime_path}")
  endif()
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    APPEND PROPERTY AUTOCMAKE_EXPORTED_TARGETS "${name}")
  install(
    TARGETS ${name}
    EXPORT ${AUTOCMAKE_EXPORT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endfunction()

# @brief Add a namespaced alias when it does not already exist.
function(_autocmake_add_alias alias target)
  if(NOT TARGET "${alias}")
    add_library("${alias}" ALIAS "${target}")
  endif()
endfunction()
