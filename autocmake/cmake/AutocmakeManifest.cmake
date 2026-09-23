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

# @brief Register one resource in the install-prefix index.
#
# Marker path: share/autocmake_index/resource_index/<type>/<package>.
# The file name is the package name, matching ament_index.
function(_autocmake_index resource_type)
  if(resource_type STREQUAL "")
    message(FATAL_ERROR "_autocmake_index: resource_type is empty")
  endif()
  cmake_parse_arguments(ARG "" "PACKAGE_NAME" "CONTENT" ${ARGN})
  if(NOT ARG_PACKAGE_NAME)
    set(ARG_PACKAGE_NAME "${PROJECT_NAME}")
  endif()
  set(_dest "share/autocmake_index/resource_index/${resource_type}")
  set(_marker "${CMAKE_BINARY_DIR}/autocmake_index/${_dest}/${ARG_PACKAGE_NAME}")
  get_filename_component(_marker_dir "${_marker}" DIRECTORY)
  file(MAKE_DIRECTORY "${_marker_dir}")
  if(ARG_CONTENT)
    file(WRITE "${_marker}" "${ARG_CONTENT}")
  else()
    file(WRITE "${_marker}" "")
  endif()
  install(FILES "${_marker}" DESTINATION "${_dest}")
endfunction()

# @brief Register this package under the "packages" resource type.
function(_autocmake_index_pkg)
  _autocmake_index(packages ${ARGN})
endfunction()

# @brief Parse package.xml into ${PROJECT_NAME}_VERSION and dependency lists.
#
# The project() name must match <name>. When project() set VERSION, it must
# match <version>. Lists follow ament package format 3:
# BUILD_DEPENDS, BUILDTOOL_DEPENDS, EXPORT_DEPENDS, TEST_DEPENDS.
# <depend> is both a build depend and an export depend.
macro(autocmake_xml)
  if(NOT PROJECT_NAME OR PROJECT_NAME STREQUAL "Project")
    message(FATAL_ERROR "autocmake_xml: call project() first")
  endif()
  get_property(_autocmake_xml_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_XML_DONE)
  if(_autocmake_xml_done)
    message(FATAL_ERROR "autocmake_xml() already ran in this directory")
  endif()
  _autocmake_parse(_autocmake_xml "" "DIRECTORY" "" ${ARGN})
  if(_autocmake_xml_DIRECTORY)
    set(_autocmake_xml_dir "${_autocmake_xml_DIRECTORY}")
  else()
    set(_autocmake_xml_dir "${PROJECT_SOURCE_DIR}")
  endif()
  if(NOT EXISTS "${_autocmake_xml_dir}/package.xml")
    message(FATAL_ERROR "autocmake_xml: ${_autocmake_xml_dir}/package.xml does not exist")
  endif()
  if(NOT AUTOCMAKE_DIR)
    message(FATAL_ERROR "autocmake_xml: AUTOCMAKE_DIR is not set; find_package(autocmake) first")
  endif()
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/autocmake_package")
  set(_autocmake_xml_cmake "${CMAKE_CURRENT_BINARY_DIR}/autocmake_package/package.cmake")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}"
      "${AUTOCMAKE_DIR}/cmake/package_xml.py"
      --cmake "${_autocmake_xml_dir}/package.xml" "${_autocmake_xml_cmake}"
    RESULT_VARIABLE _autocmake_xml_rc
    ERROR_VARIABLE _autocmake_xml_err)
  if(NOT _autocmake_xml_rc EQUAL 0)
    message(FATAL_ERROR "autocmake_xml: ${_autocmake_xml_err}")
  endif()
  include("${_autocmake_xml_cmake}")
  if(NOT _AUTOCMAKE_PACKAGE_NAME STREQUAL PROJECT_NAME)
    message(FATAL_ERROR
      "autocmake_xml: package.xml name '${_AUTOCMAKE_PACKAGE_NAME}' does not match PROJECT_NAME '${PROJECT_NAME}'")
  endif()
  if(PROJECT_VERSION AND NOT PROJECT_VERSION STREQUAL "${${PROJECT_NAME}_VERSION}")
    message(FATAL_ERROR
      "autocmake_xml: project() version '${PROJECT_VERSION}' does not match package.xml version '${${PROJECT_NAME}_VERSION}'")
  endif()
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_XML_DONE TRUE)
endmacro()

# @brief Parse package.xml once for this directory when the file exists.
macro(_autocmake_ensure_xml)
  get_property(_autocmake_xml_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_XML_DONE)
  if(NOT _autocmake_xml_done AND EXISTS "${PROJECT_SOURCE_DIR}/package.xml")
    autocmake_xml()
  endif()
  get_property(_autocmake_xml_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_XML_DONE)
endmacro()

# @brief Read MAJOR.MINOR.PATCH from a package.xml file.
#
# Sets <prefix>_VERSION, <prefix>_VERSION_MAJOR, <prefix>_VERSION_MINOR,
# and <prefix>_VERSION_PATCH in the caller.
function(autocmake_package_version package_xml prefix)
  if(NOT Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
  endif()
  execute_process(
    COMMAND "${Python3_EXECUTABLE}"
      "${AUTOCMAKE_DIR}/tools/print_package_xml_version.py" "${package_xml}"
    OUTPUT_VARIABLE _autocmake_package_version
    ERROR_VARIABLE _autocmake_package_version_error
    RESULT_VARIABLE _autocmake_package_version_result)
  if(NOT _autocmake_package_version_result EQUAL 0)
    message(FATAL_ERROR "autocmake_package_version: ${_autocmake_package_version_error}")
  endif()
  string(REPLACE "." ";" _autocmake_package_version_list "${_autocmake_package_version}")
  list(GET _autocmake_package_version_list 0 _autocmake_package_version_major)
  list(GET _autocmake_package_version_list 1 _autocmake_package_version_minor)
  list(GET _autocmake_package_version_list 2 _autocmake_package_version_patch)
  set(${prefix}_VERSION "${_autocmake_package_version}" PARENT_SCOPE)
  set(${prefix}_VERSION_MAJOR "${_autocmake_package_version_major}" PARENT_SCOPE)
  set(${prefix}_VERSION_MINOR "${_autocmake_package_version_minor}" PARENT_SCOPE)
  set(${prefix}_VERSION_PATCH "${_autocmake_package_version_patch}" PARENT_SCOPE)
endfunction()

# @brief Re-export packages so downstream config files call find_dependency.
#
# Call before autocmake_package(). Names must be valid find_package() arguments.
macro(autocmake_export)
  get_property(_autocmake_package_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PACKAGE_DONE)
  if(_autocmake_package_done)
    message(FATAL_ERROR "autocmake_export() must be called before autocmake_package()")
  endif()
  foreach(_autocmake_export_pkg IN ITEMS ${ARGN})
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES
      "find_dependency(${_autocmake_export_pkg})")
  endforeach()
endmacro()

# @brief Read one package.xml version constraint: kind and version, or empty.
function(_autocmake_version_spec dep out_kind out_version)
  set(${out_kind} "" PARENT_SCOPE)
  set(${out_version} "" PARENT_SCOPE)
  set(_specs "${${PROJECT_NAME}_VERSION_SPECS}")
  list(LENGTH _specs _count)
  if(NOT _count)
    return()
  endif()
  math(EXPR _last "${_count} - 1")
  foreach(_index RANGE 0 ${_last} 3)
    list(GET _specs ${_index} _name)
    if(_name STREQUAL dep)
      math(EXPR _kind_index "${_index} + 1")
      math(EXPR _version_index "${_index} + 2")
      list(GET _specs ${_kind_index} _kind)
      list(GET _specs ${_version_index} _version)
      set(${out_kind} "${_kind}" PARENT_SCOPE)
      set(${out_version} "${_version}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
endfunction()

# @brief Extra config lines for bounds find_package cannot express.
function(_autocmake_version_guard dep out_guard)
  _autocmake_version_spec("${dep}" _kind _version)
  set(_guard "")
  if(_kind STREQUAL "gt")
    set(_guard "if(${dep}_VERSION VERSION_LESS_EQUAL \"${_version}\")\n  message(FATAL_ERROR \"${dep} must be > ${_version}\")\nendif()")
  elseif(_kind STREQUAL "lt")
    set(_guard "if(NOT ${dep}_VERSION VERSION_LESS \"${_version}\")\n  message(FATAL_ERROR \"${dep} must be < ${_version}\")\nendif()")
  elseif(_kind STREQUAL "lte")
    set(_guard "if(${dep}_VERSION VERSION_GREATER \"${_version}\")\n  message(FATAL_ERROR \"${dep} must be <= ${_version}\")\nendif()")
  endif()
  set(${out_guard} "${_guard}" PARENT_SCOPE)
endfunction()

# @brief find_package one manifest dependency, including its version constraint.
function(_autocmake_find_declared dep)
  _autocmake_version_spec("${dep}" _kind _version)
  set(_args REQUIRED)
  if(ARGV1)
    list(APPEND _args PRIVATE)
  endif()
  if(_kind STREQUAL "eq")
    list(APPEND _args EXACT VERSION "${_version}")
  elseif(_kind STREQUAL "gte" OR _kind STREQUAL "gt")
    list(APPEND _args VERSION "${_version}")
  endif()
  autocmake_find(${dep} ${_args})
  if(${dep}_FOUND AND (_kind STREQUAL "gt" OR _kind STREQUAL "lt" OR _kind STREQUAL "lte"))
    _autocmake_version_guard("${dep}" _guard)
    if(_kind STREQUAL "gt" AND NOT ${${dep}_VERSION} VERSION_GREATER "${_version}")
      autocmake_error("${dep} ${${dep}_VERSION} does not satisfy > ${_version}")
    elseif(_kind STREQUAL "lt" AND NOT ${${dep}_VERSION} VERSION_LESS "${_version}")
      autocmake_error("${dep} ${${dep}_VERSION} does not satisfy < ${_version}")
    elseif(_kind STREQUAL "lte" AND ${${dep}_VERSION} VERSION_GREATER "${_version}")
      autocmake_error("${dep} ${${dep}_VERSION} does not satisfy <= ${_version}")
    endif()
    if(NOT ARGV1 AND _guard)
      set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES "${_guard}")
    endif()
  endif()
endfunction()

# @brief Export one manifest dependency that this package does not find itself.
function(_autocmake_export_spec dep)
  _autocmake_version_spec("${dep}" _kind _version)
  set(_call "find_dependency(${dep}")
  if(_kind STREQUAL "eq")
    string(APPEND _call " ${_version} EXACT")
  elseif(_kind STREQUAL "gte" OR _kind STREQUAL "gt")
    string(APPEND _call " ${_version}")
  endif()
  string(APPEND _call ")")
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES "${_call}")
  _autocmake_version_guard("${dep}" _guard)
  if(_guard)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES "${_guard}")
  endif()
endfunction()

# @brief find_package every build dependency declared in package.xml.
#
# buildtool_depend on autocmake is skipped. <depend> is found and
# exported. build_depend is private. exec_depend and build_export_depend are
# exported without a find. REQUIRED names must appear in the manifest.
macro(autocmake_dependencies)
  get_property(_autocmake_dependencies_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_DEPENDENCIES_DONE)
  if(_autocmake_dependencies_done)
    message(FATAL_ERROR "autocmake_dependencies() already ran")
  endif()
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_DEPENDENCIES_DONE TRUE)
  get_property(_autocmake_package_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PACKAGE_DONE)
  if(_autocmake_package_done)
    message(FATAL_ERROR "autocmake_dependencies() must be called before autocmake_package()")
  endif()
  _autocmake_parse(_autocmake_fbd "" "" "REQUIRED" ${ARGN})
  _autocmake_ensure_xml()
  foreach(_autocmake_req IN LISTS _autocmake_fbd_REQUIRED)
    if(NOT _autocmake_req IN_LIST ${PROJECT_NAME}_BUILD_DEPENDS
        AND NOT _autocmake_req IN_LIST ${PROJECT_NAME}_BUILDTOOL_DEPENDS
        AND NOT _autocmake_req IN_LIST ${PROJECT_NAME}_EXPORT_DEPENDS)
      message(FATAL_ERROR
        "autocmake_dependencies: '${_autocmake_req}' is not listed in package.xml")
    endif()
  endforeach()
  foreach(_autocmake_pkg IN LISTS ${PROJECT_NAME}_BUILDTOOL_DEPENDS ${PROJECT_NAME}_BUILD_DEPENDS)
    if(_autocmake_pkg STREQUAL "autocmake")
    elseif(_autocmake_pkg IN_LIST ${PROJECT_NAME}_EXPORT_DEPENDS)
      _autocmake_find_declared("${_autocmake_pkg}" FALSE)
    else()
      _autocmake_find_declared("${_autocmake_pkg}" TRUE)
    endif()
  endforeach()
  foreach(_autocmake_pkg IN LISTS ${PROJECT_NAME}_EXPORT_DEPENDS)
    if(NOT _autocmake_pkg IN_LIST ${PROJECT_NAME}_BUILD_DEPENDS
        AND NOT _autocmake_pkg IN_LIST ${PROJECT_NAME}_BUILDTOOL_DEPENDS)
      _autocmake_export_spec("${_autocmake_pkg}")
    endif()
  endforeach()
endmacro()

# @brief Finish one package: manifest, index, CMake config. Call once, at the end.
macro(autocmake_package)
  get_property(_autocmake_package_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PACKAGE_DONE)
  if(_autocmake_package_done)
    message(FATAL_ERROR "autocmake_package() must be called exactly once")
  endif()
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PACKAGE_DONE TRUE)
  _autocmake_ensure_xml()
  install(
    FILES "${PROJECT_SOURCE_DIR}/package.xml"
    DESTINATION "share/${PROJECT_NAME}")
  _autocmake_index_pkg()
  if(NOT ${PROJECT_NAME}_BUILD_TYPE OR ${PROJECT_NAME}_BUILD_TYPE STREQUAL "autocmake")
    _autocmake_index(autocmake)
  endif()
  _autocmake_env()
  autocmake_install()
endmacro()
