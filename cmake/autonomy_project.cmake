# @file autonomy_project.cmake
# @brief Project bootstrap: C++ standard, version.json, compile flags, testing,
#        and module paths (gz_configure_project).
#
# find_package and product options stay in the root CMakeLists.

include_guard(GLOBAL)


#[=======================================================================[.rst:
read_version_from_json
----------------------

Read version fields from a version.json file and set CMake variables.

Synopsis
^^^^^^^^

.. code-block:: cmake

  read_version_from_json(<prefix> <version_file>)

Description
^^^^^^^^^^^

Reads version information from the given JSON file and sets:

- ``<prefix>_MAJOR_VERSION`` - major version
- ``<prefix>_MINOR_VERSION`` - minor version
- ``<prefix>_PATCH_VERSION`` - patch version
- ``<prefix>_VERSION`` - full version (major.minor.patch)
- ``<prefix>_SOVERSION`` - shared-library soversion (major.minor)
- ``<prefix>_NAME`` - project name (if present in JSON)

Arguments
^^^^^^^^^

``<prefix>``
  Variable prefix, e.g. "AUTONOMY" or "AUTOLINK"

``<version_file>``
  Path to version.json (relative or absolute)

Example
^^^^^^^

.. code-block:: cmake

  read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")
  message(STATUS "${AUTONOMY_NAME} version: ${AUTONOMY_VERSION}")

version.json format
^^^^^^^^^^^^^^^^^^^

.. code-block:: json

  {
    "version": "0.0.3",
    "major": 0,
    "minor": 0,
    "patch": 3,
    "name": "Autonomy",
    "description": "Autonomous Robot Development Framework",
    "build_date": "2025-11-20"
  }

#]=======================================================================]

function(read_version_from_json PREFIX VERSION_FILE)
  if(NOT EXISTS "${VERSION_FILE}")
    message(FATAL_ERROR "Version file not found: ${VERSION_FILE}")
  endif()

  file(READ "${VERSION_FILE}" VERSION_JSON)
  if(NOT VERSION_JSON)
    message(FATAL_ERROR "Version file is empty: ${VERSION_FILE}")
  endif()

  string(REGEX MATCH "\"major\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${VERSION_JSON}")
  if(NOT DEFINED CMAKE_MATCH_1)
    message(FATAL_ERROR "Cannot find 'major' field in ${VERSION_FILE}")
  endif()
  set(_MAJOR ${CMAKE_MATCH_1})
  set(${PREFIX}_MAJOR_VERSION ${_MAJOR} PARENT_SCOPE)

  string(REGEX MATCH "\"minor\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${VERSION_JSON}")
  if(NOT DEFINED CMAKE_MATCH_1)
    message(FATAL_ERROR "Cannot find 'minor' field in ${VERSION_FILE}")
  endif()
  set(_MINOR ${CMAKE_MATCH_1})
  set(${PREFIX}_MINOR_VERSION ${_MINOR} PARENT_SCOPE)

  string(REGEX MATCH "\"patch\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${VERSION_JSON}")
  if(NOT DEFINED CMAKE_MATCH_1)
    message(FATAL_ERROR "Cannot find 'patch' field in ${VERSION_FILE}")
  endif()
  set(_PATCH ${CMAKE_MATCH_1})
  set(${PREFIX}_PATCH_VERSION ${_PATCH} PARENT_SCOPE)

  string(REGEX MATCH "\"name\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _ "${VERSION_JSON}")
  if(CMAKE_MATCH_1)
    set(_NAME ${CMAKE_MATCH_1})
    set(${PREFIX}_NAME ${_NAME} PARENT_SCOPE)
  else()
    set(_NAME ${PREFIX})
    set(${PREFIX}_NAME ${PREFIX} PARENT_SCOPE)
  endif()

  string(REGEX MATCH "\"description\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _ "${VERSION_JSON}")
  if(CMAKE_MATCH_1)
    set(${PREFIX}_DESCRIPTION ${CMAKE_MATCH_1} PARENT_SCOPE)
  endif()

  string(REGEX MATCH "\"build_date\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _ "${VERSION_JSON}")
  if(CMAKE_MATCH_1)
    set(${PREFIX}_BUILD_DATE ${CMAKE_MATCH_1} PARENT_SCOPE)
  endif()

  set(_VERSION "${_MAJOR}.${_MINOR}.${_PATCH}")
  set(${PREFIX}_VERSION ${_VERSION} PARENT_SCOPE)

  set(_SOVERSION "${_MAJOR}.${_MINOR}")
  set(${PREFIX}_SOVERSION ${_SOVERSION} PARENT_SCOPE)

  if(NOT ${PREFIX}_VERSION_QUIET)
    message(STATUS "${_NAME} version: ${_VERSION}")
  endif()
endfunction()

# @brief Generate autonomy-config-version.cmake and @c autonomy/common/version.cpp.
# @details Sets parent-scope @c AUTONOMY_VERSION_CPP; optionally reads Git metadata.
function(autonomy_configure_version)
  if(DEFINED AUTONOMY_VERSION)
    set(PROJECT_VERSION "${AUTONOMY_VERSION}")
    set(PROJECT_VERSION "${AUTONOMY_VERSION}" PARENT_SCOPE)
    set(CMAKE_PROJECT_VERSION "${AUTONOMY_VERSION}" PARENT_SCOPE)
  endif()

  configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/autonomy_config_version.cmake.in"
    "${PROJECT_BINARY_DIR}/autonomy-config-version.cmake" @ONLY)

  find_package(Git QUIET)

  if(DEFINED GIT_COMMIT_ID OR DEFINED GIT_COMMIT_DATE)
    # Caller-provided overrides.
  elseif(Git_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(COMMAND
        "${GIT_EXECUTABLE}" rev-parse --short HEAD
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_ID
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%an
        OUTPUT_VARIABLE GIT_COMMIT_AUTHOR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%ae
        OUTPUT_VARIABLE GIT_COMMIT_EMAIL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:"%ad" --date=format:"%Y-%m-%d %H:%M:%S"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    set_property(
        DIRECTORY APPEND
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${PROJECT_SOURCE_DIR}/.git/index"
    )
  else()
    set(GIT_BRANCH "unknown")
    set(GIT_COMMIT_ID "Unknown")
    set(GIT_COMMIT_DATE "Unknown")
    set(GIT_COMMIT_AUTHOR "unknown")
    set(GIT_COMMIT_EMAIL "unknown")
    set(GIT_VERSION "unknown")
  endif()

  string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)

  if(UNIX OR WIN32)
    execute_process(
        COMMAND hostname
        OUTPUT_VARIABLE BUILD_HOST
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    execute_process(
        COMMAND whoami
        OUTPUT_VARIABLE BUILD_USER
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
  endif()
  if(NOT BUILD_HOST)
    set(BUILD_HOST "unknown")
  endif()
  if(NOT BUILD_USER)
    set(BUILD_USER "unknown")
  endif()

  if(CMAKE_SYSTEM_NAME)
    set(SYSTEM_NAME ${CMAKE_SYSTEM_NAME})
  else()
    set(SYSTEM_NAME "unknown")
  endif()
  if(CMAKE_SYSTEM_PROCESSOR)
    set(SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
  else()
    set(SYSTEM_PROCESSOR "unknown")
  endif()
  if(CMAKE_SYSTEM_VERSION)
    set(SYSTEM_VERSION ${CMAKE_SYSTEM_VERSION})
  else()
    set(SYSTEM_VERSION "unknown")
  endif()
  if(CMAKE_CXX_COMPILER_ID)
    set(COMPILER_ID ${CMAKE_CXX_COMPILER_ID})
  else()
    set(COMPILER_ID "unknown")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION)
    set(COMPILER_VERSION ${CMAKE_CXX_COMPILER_VERSION})
  else()
    set(COMPILER_VERSION "unknown")
  endif()

  message(STATUS
    "Autonomy ${AUTONOMY_VERSION} (${GIT_VERSION}, ${GIT_BRANCH}) "
    "${COMPILER_ID} ${COMPILER_VERSION} on ${SYSTEM_NAME}/${SYSTEM_PROCESSOR}")

  configure_file(
      "${PROJECT_SOURCE_DIR}/autonomy/common/version.cpp.in"
      "${PROJECT_BINARY_DIR}/autonomy/common/version.cpp"
      @ONLY)
  set(AUTONOMY_VERSION_CPP "${PROJECT_BINARY_DIR}/autonomy/common/version.cpp"
      PARENT_SCOPE)
endfunction()

# @brief Entry for root and domain modules: CXX, version, autonomy_common,
#        Cartographer config paths.
macro(autonomy_configure_project)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

  set(AUTONOMY_WORKSPACE_ROOT "${PROJECT_SOURCE_DIR}")

  read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")

  include("${PROJECT_SOURCE_DIR}/cmake/autonomy_common.cmake")
  autonomy_initialize_project()
  autonomy_enable_testing()
  # CMAKE_MODULE_PATH for cmake/ is usually set by the root CMakeLists before
  # this macro; keep appending for callers that only invoke configure_project.
  list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
  list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

  set(_autonomy_config_dir ${CMAKE_INSTALL_PREFIX}/share/autonomy)
  set(AUTONOMY_CONFIGURATION_FILES_DIRECTORY
    ${_autonomy_config_dir}/localization/conf/cartographer
    CACHE PATH "Cartographer / module conf install root")
  set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY
    ${_autonomy_config_dir}/localization/conf/cartographer
    CACHE PATH ".lua configuration files directory for cartographer")
  unset(_autonomy_config_dir)
endmacro()
