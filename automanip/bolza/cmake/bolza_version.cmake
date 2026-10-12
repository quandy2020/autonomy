# Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#[=======================================================================[.rst:
bolza_collect_git_version
-------------------------

Read ``version.json``, optionally enrich with ``git describe`` (tags
``bolza-vX.Y.Z``), and export:

* ``BOLZA_VERSION`` / ``MAJOR`` / ``MINOR`` / ``PATCH`` / ``SOVERSION``
* ``BOLZA_PACKAGE_NAME`` / ``BOLZA_PACKAGE_DESCRIPTION``
* ``BOLZA_FULL_VERSION`` / git metadata / build date
#]=======================================================================]

function(bolza_collect_git_version)
  set(_root "${BOLZA_ROOT_DIR}")
  set(_version_file "${_root}/version.json")
  if(NOT EXISTS "${_version_file}")
    message(FATAL_ERROR "bolza: missing ${_version_file}")
  endif()

  file(READ "${_version_file}" _json)
  string(JSON _name ERROR_VARIABLE _json_err GET "${_json}" name)
  if(_json_err)
    set(_name "Bolza")
    set(_desc "Bolza-form optimal control library")
    set(_major "0")
    set(_minor "1")
    set(_patch "0")
  else()
    string(JSON _desc GET "${_json}" description)
    string(JSON _major GET "${_json}" major)
    string(JSON _minor GET "${_json}" minor)
    string(JSON _patch GET "${_json}" patch)
  endif()

  set(_git_describe "unknown")
  set(_git_commit "unknown")
  set(_git_branch "unknown")
  set(_git_commit_date "unknown")
  set(_git_dirty "false")

  find_package(Git QUIET)
  if(Git_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty --match "bolza-v*"
      WORKING_DIRECTORY "${_root}"
      OUTPUT_VARIABLE _desc_pkg
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(_desc_pkg STREQUAL "")
      execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
        WORKING_DIRECTORY "${_root}"
        OUTPUT_VARIABLE _desc_pkg
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    endif()
    if(NOT _desc_pkg STREQUAL "")
      set(_git_describe "${_desc_pkg}")
    endif()
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse --short=8 HEAD
      WORKING_DIRECTORY "${_root}"
      OUTPUT_VARIABLE _git_commit
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
      WORKING_DIRECTORY "${_root}"
      OUTPUT_VARIABLE _git_branch
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} show -s --format=%ci HEAD
      WORKING_DIRECTORY "${_root}"
      OUTPUT_VARIABLE _git_commit_date
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} status --porcelain
      WORKING_DIRECTORY "${_root}"
      OUTPUT_VARIABLE _dirty_out
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
    if(NOT _dirty_out STREQUAL "")
      set(_git_dirty "true")
    endif()
    if(_git_describe MATCHES "^bolza-v([0-9]+)\\.([0-9]+)\\.([0-9]+)")
      set(_major "${CMAKE_MATCH_1}")
      set(_minor "${CMAKE_MATCH_2}")
      set(_patch "${CMAKE_MATCH_3}")
    elseif(_git_describe MATCHES "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)")
      set(_major "${CMAKE_MATCH_1}")
      set(_minor "${CMAKE_MATCH_2}")
      set(_patch "${CMAKE_MATCH_3}")
    endif()
  endif()

  set(_version "${_major}.${_minor}.${_patch}")
  set(_full "${_version}")
  if(NOT _git_commit STREQUAL "unknown")
    set(_full "${_version}+g${_git_commit}")
    if(_git_dirty STREQUAL "true")
      set(_full "${_full}-dirty")
    endif()
  endif()

  string(TIMESTAMP _build_date "%Y-%m-%d")
  string(TIMESTAMP _build_time "%Y-%m-%d %H:%M:%S")

  set(BOLZA_PACKAGE_NAME "${_name}" PARENT_SCOPE)
  set(BOLZA_PACKAGE_DESCRIPTION "${_desc}" PARENT_SCOPE)
  set(BOLZA_VERSION "${_version}" PARENT_SCOPE)
  set(BOLZA_VERSION_MAJOR "${_major}" PARENT_SCOPE)
  set(BOLZA_VERSION_MINOR "${_minor}" PARENT_SCOPE)
  set(BOLZA_VERSION_PATCH "${_patch}" PARENT_SCOPE)
  set(BOLZA_SOVERSION "${_major}.${_minor}" PARENT_SCOPE)
  set(BOLZA_FULL_VERSION "${_full}" PARENT_SCOPE)
  set(BOLZA_GIT_DESCRIBE "${_git_describe}" PARENT_SCOPE)
  set(BOLZA_GIT_COMMIT "${_git_commit}" PARENT_SCOPE)
  set(BOLZA_GIT_BRANCH "${_git_branch}" PARENT_SCOPE)
  set(BOLZA_GIT_COMMIT_DATE "${_git_commit_date}" PARENT_SCOPE)
  set(BOLZA_GIT_DIRTY "${_git_dirty}" PARENT_SCOPE)
  set(BOLZA_BUILD_DATE "${_build_date}" PARENT_SCOPE)
  set(BOLZA_BUILD_TIME "${_build_time}" PARENT_SCOPE)

  message(STATUS "bolza version: ${_full} (branch ${_git_branch})")
endfunction()
