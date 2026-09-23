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

set(_AUTOCMAKE_EXPORT_CONFIG_IN "${CMAKE_CURRENT_LIST_DIR}/autocmake-export-config.cmake.in")
set(_AUTOCMAKE_UNINSTALL_IN "${CMAKE_CURRENT_LIST_DIR}/autocmake-uninstall.cmake.in")

# @brief Install extra config snippets beside the package file.
macro(_autocmake_install_config_blocks)
  set(AUTOCMAKE_CONFIG_EXTRAS_BLOCK "")
  get_property(_autocmake_extras DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_CONFIG_EXTRAS)
  foreach(_extra IN LISTS _autocmake_extras)
    get_filename_component(_extra_name "${_extra}" NAME)
    if(_extra_name MATCHES "\\.cmake\\.in$")
      string(REGEX REPLACE "\\.in$" "" _extra_name "${_extra_name}")
      configure_file("${_extra}" "${CMAKE_CURRENT_BINARY_DIR}/${_extra_name}" @ONLY)
      set(_extra "${CMAKE_CURRENT_BINARY_DIR}/${_extra_name}")
    endif()
    install(FILES "${_extra}" DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}")
    get_filename_component(_extra_name "${_extra}" NAME)
    string(APPEND AUTOCMAKE_CONFIG_EXTRAS_BLOCK
      "include(\"\${CMAKE_CURRENT_LIST_DIR}/${_extra_name}\")\n")
  endforeach()
endmacro()

# @brief Install headers, write the package config, and add an uninstall target.
#
# Call once, after autocmake_build(). autocmake_package() calls this.
macro(autocmake_install)
  get_property(_autocmake_packages_created DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PACKAGES_CREATED)
  if(_autocmake_packages_created)
    message(FATAL_ERROR "autocmake_install() already ran")
  endif()
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PACKAGES_CREATED TRUE)
  if(NOT AUTOCMAKE_EXPORT_NAME)
    message(FATAL_ERROR "autocmake_install: call autocmake_project() first")
  endif()

  if(EXISTS "${PROJECT_SOURCE_DIR}/include")
    install(
      DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
      DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
      FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hh"
        PATTERN "*.hpp"
        PATTERN "*.hxx"
        PATTERN "*.inl"
        PATTERN "*.ipp"
        PATTERN "*.tpp")
  endif()

  get_property(_autocmake_dependencies DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES)
  if(NOT _autocmake_dependencies)
    set(AUTOCMAKE_FIND_DEPENDENCY_CALLS "")
  else()
    list(REMOVE_DUPLICATES _autocmake_dependencies)
    string(REPLACE ";" "\n" AUTOCMAKE_FIND_DEPENDENCY_CALLS "${_autocmake_dependencies}")
  endif()

  _autocmake_install_config_blocks()

  set(AUTOCMAKE_TARGETS_FILE "${AUTOCMAKE_EXPORT_NAME}.cmake")
  get_property(_autocmake_exported DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    PROPERTY AUTOCMAKE_EXPORTED_TARGETS)
  if(_autocmake_exported)
    set(AUTOCMAKE_TARGETS_INCLUDE
      "include(\"\${CMAKE_CURRENT_LIST_DIR}/${AUTOCMAKE_TARGETS_FILE}\")")
    install(
      EXPORT ${AUTOCMAKE_EXPORT_NAME}
      DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}"
      FILE "${AUTOCMAKE_TARGETS_FILE}"
      NAMESPACE ${PROJECT_NAME}::)
  else()
    set(AUTOCMAKE_TARGETS_INCLUDE "")
  endif()

  configure_package_config_file(
    "${_AUTOCMAKE_EXPORT_CONFIG_IN}"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    INSTALL_DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}"
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion)

  install(
    FILES
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}")

  if(NOT TARGET uninstall)
    configure_file(
      "${_AUTOCMAKE_UNINSTALL_IN}"
      "${CMAKE_CURRENT_BINARY_DIR}/autocmake-uninstall.cmake"
      @ONLY)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_BINARY_DIR}/autocmake-uninstall.cmake")
  endif()

  if(CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Build configurations: ${CMAKE_CONFIGURATION_TYPES}")
  else()
    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
  endif()
  message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
endmacro()
