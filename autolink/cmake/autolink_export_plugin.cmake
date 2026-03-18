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

# ============================================================================
# autolink_export_plugin
# ============================================================================
#
# Export a plugin library and its description file for autolink plugin system.
#
# Usage:
#   autolink_export_plugin(
#     LIBRARY <target_name>
#     DESCRIPTION_FILE <xml_file>
#     [INDEX_NAME <index_name>]
#     [INSTALL_SUBDIR <subdir>]
#   )
#
# Arguments:
#   LIBRARY: CMake target name of the plugin library (required)
#   DESCRIPTION_FILE: Path to the plugin description XML file (required)
#   INDEX_NAME: Name for the plugin index file (default: same as library target name)
#   INSTALL_SUBDIR: Subdirectory for installation (default: pluginlib)
#
# Example:
#   add_library(my_plugin SHARED my_plugin.cpp)
#   autolink_export_plugin(
#     LIBRARY my_plugin
#     DESCRIPTION_FILE my_plugin.xml
#   )
#
function(autolink_export_plugin)
  cmake_parse_arguments(_ARG
    ""
    "LIBRARY;DESCRIPTION_FILE;INDEX_NAME;INSTALL_SUBDIR"
    ""
    ${ARGN}
  )

  # Validate required arguments
  if(NOT _ARG_LIBRARY)
    message(FATAL_ERROR "autolink_export_plugin() requires LIBRARY argument")
  endif()

  if(NOT _ARG_DESCRIPTION_FILE)
    message(FATAL_ERROR "autolink_export_plugin() requires DESCRIPTION_FILE argument")
  endif()

  # Check if library target exists
  if(NOT TARGET ${_ARG_LIBRARY})
    message(FATAL_ERROR "autolink_export_plugin() library target '${_ARG_LIBRARY}' does not exist")
  endif()

  # Check if description file exists
  # Support both absolute and relative paths
  if(IS_ABSOLUTE "${_ARG_DESCRIPTION_FILE}")
    set(_desc_file_path "${_ARG_DESCRIPTION_FILE}")
  else()
    set(_desc_file_path "${CMAKE_CURRENT_SOURCE_DIR}/${_ARG_DESCRIPTION_FILE}")
  endif()
  
  if(NOT EXISTS "${_desc_file_path}")
    message(FATAL_ERROR "autolink_export_plugin() description file '${_ARG_DESCRIPTION_FILE}' does not exist at '${_desc_file_path}'")
  endif()

  # Set defaults
  if(NOT _ARG_INDEX_NAME)
    set(_ARG_INDEX_NAME "${_ARG_LIBRARY}")
  endif()

  if(NOT _ARG_INSTALL_SUBDIR)
    set(_ARG_INSTALL_SUBDIR "pluginlib")
  endif()

  # Get library output name
  get_target_property(_lib_output_name ${_ARG_LIBRARY} OUTPUT_NAME)
  if(NOT _lib_output_name)
    set(_lib_output_name "${_ARG_LIBRARY}")
  endif()

  # Get library output directory
  get_target_property(_lib_output_dir ${_ARG_LIBRARY} LIBRARY_OUTPUT_DIRECTORY)
  if(NOT _lib_output_dir)
    get_target_property(_lib_output_dir ${_ARG_LIBRARY} RUNTIME_OUTPUT_DIRECTORY)
  endif()
  if(NOT _lib_output_dir)
    set(_lib_output_dir "${CMAKE_BINARY_DIR}/lib")
  endif()

  # Build directory paths
  set(_build_lib_dir "${_lib_output_dir}")
  set(_build_desc_dir "${CMAKE_BINARY_DIR}/share/autolink/${_ARG_INSTALL_SUBDIR}")
  set(_build_index_dir "${CMAKE_BINARY_DIR}/share/autolink_plugin_index")

  # Install directory paths
  set(_install_lib_dir "${CMAKE_INSTALL_PREFIX}/lib/autolink/${_ARG_INSTALL_SUBDIR}")
  set(_install_desc_dir "${CMAKE_INSTALL_PREFIX}/share/autolink/${_ARG_INSTALL_SUBDIR}")
  set(_install_index_dir "${CMAKE_INSTALL_PREFIX}/share/autolink_plugin_index")

  # Create build directories
  file(MAKE_DIRECTORY "${_build_desc_dir}")
  file(MAKE_DIRECTORY "${_build_index_dir}")

  # Configure XML description file for build
  # The library path in XML will be resolved via AUTOLINK_PLUGIN_LIB_PATH
  set(_plugin_lib_path "lib${_lib_output_name}${CMAKE_SHARED_LIBRARY_SUFFIX}")
  configure_file(
    "${_desc_file_path}"
    "${_build_desc_dir}/${_ARG_DESCRIPTION_FILE}"
    @ONLY
  )

  # Configure XML description file for install
  configure_file(
    "${_desc_file_path}"
    "${CMAKE_CURRENT_BINARY_DIR}/${_ARG_DESCRIPTION_FILE}.install"
    @ONLY
  )

  # Generate plugin index file for build: content must be path under build/share,
  # not absolute source path (DESCRIPTION_FILE may be absolute)
  get_filename_component(_desc_basename "${_ARG_DESCRIPTION_FILE}" NAME)
  file(WRITE "${_build_index_dir}/${_ARG_INDEX_NAME}"
    "${_build_desc_dir}/${_desc_basename}\n"
  )

  # Install plugin library
  install(TARGETS ${_ARG_LIBRARY}
    LIBRARY DESTINATION lib/autolink/${_ARG_INSTALL_SUBDIR}
    RUNTIME DESTINATION lib/autolink/${_ARG_INSTALL_SUBDIR}
    ARCHIVE DESTINATION lib/autolink/${_ARG_INSTALL_SUBDIR}
  )

  # Install plugin description file
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${_ARG_DESCRIPTION_FILE}.install"
    DESTINATION share/autolink/${_ARG_INSTALL_SUBDIR}
    RENAME "${_ARG_DESCRIPTION_FILE}"
  )

  # Install plugin index file
  install(FILES "${_build_index_dir}/${_ARG_INDEX_NAME}"
    DESTINATION share/autolink_plugin_index
  )

  # Store plugin information in global cache for test environment setup
  set(_plugin_info "${_ARG_LIBRARY}|${_build_lib_dir}|${_build_desc_dir}|${_build_index_dir}")
  
  # Get existing list from cache
  get_property(_existing_plugins CACHE _AUTOLINK_EXPORTED_PLUGINS PROPERTY VALUE)
  if(NOT _existing_plugins)
    set(_existing_plugins "")
  endif()
  
  # Append new plugin info
  list(APPEND _existing_plugins "${_plugin_info}")
  
  # Store back to cache
  set(_AUTOLINK_EXPORTED_PLUGINS "${_existing_plugins}" CACHE INTERNAL "List of exported plugins")

  message(STATUS "Exported plugin: ${_ARG_LIBRARY}")
  message(STATUS "  Description: ${_ARG_DESCRIPTION_FILE}")
  message(STATUS "  Index: ${_ARG_INDEX_NAME}")
endfunction()

# ============================================================================
# autolink_setup_plugin_test_environment
# ============================================================================
#
# Set up environment variables for tests to find exported plugins.
#
# Usage:
#   autolink_setup_plugin_test_environment(<test_target>)
#
# Arguments:
#   test_target: CMake target name of the test executable
#
function(autolink_setup_plugin_test_environment test_target)
  if(NOT TARGET ${test_target})
    message(FATAL_ERROR "autolink_setup_plugin_test_environment() test target '${test_target}' does not exist")
  endif()

  # Collect all plugin paths
  set(_plugin_lib_paths "")
  set(_plugin_desc_paths "")
  set(_plugin_index_paths "")

  # Get plugin list from cache
  get_property(_exported_plugins CACHE _AUTOLINK_EXPORTED_PLUGINS PROPERTY VALUE)
  if(_exported_plugins)
    foreach(_plugin_info ${_exported_plugins})
      string(REPLACE "|" ";" _parts ${_plugin_info})
      list(GET _parts 1 _lib_dir)
      list(GET _parts 2 _desc_dir)
      list(GET _parts 3 _index_dir)

      list(APPEND _plugin_lib_paths "${_lib_dir}")
      list(APPEND _plugin_desc_paths "${_desc_dir}")
      list(APPEND _plugin_index_paths "${_index_dir}")
    endforeach()
  endif()

  # Remove duplicates and join with colon
  if(_plugin_lib_paths)
    list(REMOVE_DUPLICATES _plugin_lib_paths)
    string(REPLACE ";" ":" _plugin_lib_paths "${_plugin_lib_paths}")
  endif()

  if(_plugin_desc_paths)
    list(REMOVE_DUPLICATES _plugin_desc_paths)
    string(REPLACE ";" ":" _plugin_desc_paths "${_plugin_desc_paths}")
  endif()

  if(_plugin_index_paths)
    list(REMOVE_DUPLICATES _plugin_index_paths)
    string(REPLACE ";" ":" _plugin_index_paths "${_plugin_index_paths}")
  endif()

  # Set test properties
  set(_env_vars "")
  if(_plugin_lib_paths)
    list(APPEND _env_vars "AUTOLINK_PLUGIN_LIB_PATH=${_plugin_lib_paths}:$ENV{AUTOLINK_PLUGIN_LIB_PATH}")
  endif()
  if(_plugin_desc_paths)
    list(APPEND _env_vars "AUTOLINK_PLUGIN_DESCRIPTION_PATH=${_plugin_desc_paths}:$ENV{AUTOLINK_PLUGIN_DESCRIPTION_PATH}")
  endif()
  if(_plugin_index_paths)
    list(APPEND _env_vars "AUTOLINK_PLUGIN_INDEX_PATH=${_plugin_index_paths}:$ENV{AUTOLINK_PLUGIN_INDEX_PATH}")
  endif()

  if(_env_vars)
    set_tests_properties(${test_target} PROPERTIES ENVIRONMENT "${_env_vars}")
  endif()
endfunction()
