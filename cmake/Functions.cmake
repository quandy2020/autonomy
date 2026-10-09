# Copyright 2025 The Openbot Authors (duyongquan)
#
# -----------------------------------------------------------------------------
# Autonomy umbrella helpers
#
# Included once from the package-root CMakeLists.txt. Keeps the root file as
# a thin wire-up; all reusable logic for domains / siblings / version /
# install lives here. Proto generation is in ProtoHelper.cmake.
#
# Dependencies
#   Core third-party   package.xml → autocmake_build()
#   Domain-specific    each autonomy/<mod>/CMakeLists.txt find_package()
#
# Call order (from root CMakeLists.txt)
#   autonomy_modules()
#   autonomy_embed_siblings()
#   autonomy_configure_version()
#   autonomy_add_proto()          # ProtoHelper.cmake
#   autonomy_add_domains()
#   autonomy_install_headers()
# -----------------------------------------------------------------------------

include_guard(GLOBAL)
include("${CMAKE_CURRENT_LIST_DIR}/ProtoHelper.cmake")

# --- modules -----------------------------------------------------------------
# Scan autonomy/*/CMakeLists.txt and declare AUTONOMY_BUILD_<MOD> options.
#
# Outputs
#   AUTONOMY_MODULES           all discovered domain names (sorted)
#   AUTONOMY_ENABLED_MODULES   subset with AUTONOMY_BUILD_<MOD>=ON
#
macro(autonomy_modules)
  set(AUTONOMY_MODULES "")
  set(AUTONOMY_ENABLED_MODULES "")
  file(GLOB _entries LIST_DIRECTORIES true "${PROJECT_SOURCE_DIR}/autonomy/*")
  foreach(_path IN LISTS _entries)
    get_filename_component(_name "${_path}" NAME)
    if(NOT EXISTS "${_path}/CMakeLists.txt")
      continue()
    endif()
    # Same marker as workspace discovery: skip this domain entirely.
    if(EXISTS "${_path}/AUTOCMAKE_IGNORE" OR EXISTS "${_path}/COLCON_IGNORE")
      message(STATUS "autonomy modules: skip ${_name} (AUTOCMAKE_IGNORE)")
      continue()
    endif()
    list(APPEND AUTONOMY_MODULES "${_name}")
    string(TOUPPER "${_name}" _u)
    option(AUTONOMY_BUILD_${_u} "Build autonomy ${_name}" ON)
    if(AUTONOMY_BUILD_${_u})
      list(APPEND AUTONOMY_ENABLED_MODULES "${_name}")
    endif()
  endforeach()
  list(SORT AUTONOMY_MODULES)
  list(SORT AUTONOMY_ENABLED_MODULES)
  message(STATUS "autonomy modules: ${AUTONOMY_ENABLED_MODULES}")
endmacro()

# --- embed helpers -----------------------------------------------------------
# Write a minimal <name>Config.cmake stub and prepend it to CMAKE_PREFIX_PATH
# so find_package(<name>) / package.xml resolution treats the embedded target
# as already FOUND without a prior install.
function(_autonomy_embed_config name)
  set(_dir "${CMAKE_BINARY_DIR}/_embedded_cmake/${name}")
  file(MAKE_DIRECTORY "${_dir}")
  file(WRITE "${_dir}/${name}Config.cmake" "set(${name}_FOUND TRUE)\n")
  file(WRITE "${_dir}/${name}-config.cmake" "set(${name}_FOUND TRUE)\n")
  set(CMAKE_PREFIX_PATH "${_dir};${CMAKE_PREFIX_PATH}" PARENT_SCOPE)
endfunction()

# Always embed required in-tree siblings (automsgs, autolink).
#
# Call before autocmake_build() so stub Configs are on CMAKE_PREFIX_PATH
# when package.xml peers are resolved. Pins Protobuf 3.19 first because
# both siblings (and autonomy_proto) need a consistent protoc.
macro(autonomy_embed_siblings)
  include(EnsureProtobuf319)
  autonomy_require_protobuf()

  if(NOT TARGET automsgs)
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/automsgs/CMakeLists.txt")
      message(FATAL_ERROR "automsgs missing at ${PROJECT_SOURCE_DIR}/automsgs")
    endif()
    add_subdirectory(automsgs)
    _autonomy_embed_config(automsgs)
  endif()

  if(NOT TARGET autolink)
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/autolink/CMakeLists.txt")
      message(FATAL_ERROR "autolink missing at ${PROJECT_SOURCE_DIR}/autolink")
    endif()
    # Quiet defaults when building under the umbrella.
    set(AUTOLINK_BUILD_TEST OFF CACHE BOOL "" FORCE)
    set(AUTOLINK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(AUTOLINK_BUILD_TOOLS ${BUILD_TOOLS} CACHE BOOL "" FORCE)
    set(AUTOLINK_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    option(AUTOLINK_BUILD_PYTHON "Build autolink Python bindings" ON)
    add_subdirectory(autolink)
    _autonomy_embed_config(autolink)
  endif()
endmacro()

# --- version -----------------------------------------------------------------
# Fill @VAR@ placeholders for version.cpp.in / config.hpp.cmake.
#
# Sources
#   Git          commit / describe / branch (or "unknown" without .git)
#   version.json major / minor / patch → AUTONOMY_VERSION
#   CMake        system / compiler / build timestamp
#
# Outputs
#   ${PROJECT_BINARY_DIR}/autonomy/common/version.cpp
#   ${PROJECT_BINARY_DIR}/autonomy/common/config.hpp
#   AUTONOMY_VERSION_CPP  (path consumed by autonomy/common)
macro(autonomy_configure_version)
  find_package(Git QUIET)
  set(GIT_COMMIT_ID "unknown")
  set(GIT_VERSION "unknown")
  set(GIT_BRANCH "unknown")
  if(Git_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
      OUTPUT_VARIABLE GIT_COMMIT_ID ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
      OUTPUT_VARIABLE GIT_VERSION ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
      WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
      OUTPUT_VARIABLE GIT_BRANCH ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()

  # Placeholders still referenced by version.cpp.in but not queried yet.
  set(GIT_COMMIT_DATE "Unknown")
  set(GIT_COMMIT_AUTHOR "unknown")
  set(GIT_COMMIT_EMAIL "unknown")
  string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)
  set(BUILD_HOST "unknown")
  set(BUILD_USER "unknown")
  set(SYSTEM_NAME "${CMAKE_SYSTEM_NAME}")
  set(SYSTEM_PROCESSOR "${CMAKE_SYSTEM_PROCESSOR}")
  set(SYSTEM_VERSION "${CMAKE_SYSTEM_VERSION}")
  set(COMPILER_ID "${CMAKE_CXX_COMPILER_ID}")
  set(COMPILER_VERSION "${CMAKE_CXX_COMPILER_VERSION}")

  file(READ "${PROJECT_SOURCE_DIR}/version.json" _ver_json)
  foreach(_k IN ITEMS major minor patch)
    string(REGEX MATCH "\"${_k}\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${_ver_json}")
    string(TOUPPER "${_k}" _K)
    set(AUTONOMY_${_K}_VERSION "${CMAKE_MATCH_1}")
  endforeach()
  set(AUTONOMY_VERSION
    "${AUTONOMY_MAJOR_VERSION}.${AUTONOMY_MINOR_VERSION}.${AUTONOMY_PATCH_VERSION}")

  set(AUTONOMY_VERSION_CPP "${PROJECT_BINARY_DIR}/autonomy/common/version.cpp")
  configure_file(
    "${PROJECT_SOURCE_DIR}/autonomy/common/version.cpp.in"
    "${AUTONOMY_VERSION_CPP}" @ONLY)
  configure_file(
    "${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp.cmake"
    "${PROJECT_BINARY_DIR}/autonomy/common/config.hpp" @ONLY)
endmacro()

# --- domains -----------------------------------------------------------------
# add_subdirectory each enabled domain, then build the INTERFACE umbrella.
#
# Targets
#   autonomy / autonomy::autonomy   INTERFACE, links proto + autonomy_<mod>
#   autonomy_<mod>                  created by each domain CMakeLists
#
# Also links autonomy into autolink_channel when that target exists, so the
# embedded channel stack can see generated headers under the build tree.
macro(autonomy_add_domains)
  add_library(autonomy INTERFACE)
  add_library(autonomy::autonomy ALIAS autonomy)

  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    add_subdirectory(autonomy/${_mod})
  endforeach()

  set(_deps "")
  if(TARGET autonomy_proto)
    list(APPEND _deps autonomy_proto)
  endif()
  foreach(_mod IN LISTS AUTONOMY_MODULES)
    if(TARGET autonomy_${_mod})
      list(APPEND _deps autonomy_${_mod})
    endif()
  endforeach()
  target_link_libraries(autonomy INTERFACE ${_deps})
  target_include_directories(autonomy INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>)

  if(TARGET autolink_channel)
    target_link_libraries(autolink_channel PRIVATE autonomy)
    target_include_directories(autolink_channel PRIVATE
      ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})
  endif()
endmacro()

# --- install -----------------------------------------------------------------
# Public headers per enabled domain, plus generated *.pb.h / config.hpp and
# the Find* modules under share/autonomy/cmake.
macro(autonomy_install_headers)
  foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)
    install(DIRECTORY "autonomy/${_mod}/"
      DESTINATION "include/autonomy/${_mod}"
      FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
      PATTERN "internal" EXCLUDE)
    if(IS_DIRECTORY "${PROJECT_BINARY_DIR}/autonomy/${_mod}")
      install(DIRECTORY "${PROJECT_BINARY_DIR}/autonomy/${_mod}/"
        DESTINATION "include/autonomy/${_mod}"
        FILES_MATCHING PATTERN "*.pb.h" PATTERN "*.grpc.pb.h" PATTERN "config.hpp")
    endif()
  endforeach()
  install(DIRECTORY cmake/modules DESTINATION share/autonomy/cmake)
endmacro()
