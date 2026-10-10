# Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
automanip_collect_git_version
------------------------------

Read base semver / metadata from ``version.json``, enrich with ``git describe``,
rewrite ``version.json`` (source + build tree), and export CMake variables:

* ``AUTOMANIP_VERSION`` / ``MAJOR`` / ``MINOR`` / ``PATCH`` / ``SOVERSION``
* ``AUTOMANIP_PACKAGE_NAME`` / ``AUTOMANIP_PACKAGE_DESCRIPTION``
* ``AUTOMANIP_GIT_DESCRIBE`` / ``COMMIT`` / ``BRANCH`` / ``COMMIT_DATE`` /
  ``AUTHOR`` / ``EMAIL`` / ``DIRTY``
* ``AUTOMANIP_FULL_VERSION`` — e.g. ``0.1.0+g63b8d64e-dirty``
* ``AUTOMANIP_BUILD_DATE`` / ``BUILD_TIME`` — local ``YYYY-MM-DD`` / ``YYYY-MM-DD HH:MM:SS.mmm``

Tag convention: ``automanip-vX.Y.Z`` preferred; else nearest ``vX.Y.Z``.
Either form updates major/minor/patch so CLI ``version`` matches ``describe``.
#]=======================================================================]

function(automanip_collect_git_version)
  set(_root "${AUTOMANIP_ROOT_DIR}")
  set(_version_file "${_root}/version.json")
  if(NOT EXISTS "${_version_file}")
    message(FATAL_ERROR "automanip: missing ${_version_file}")
  endif()

  file(READ "${_version_file}" _json)
  # Recover from unresolved merge conflicts (<<<<<<< / ======= / >>>>>>>).
  if(_json MATCHES "<<<<<<<" OR _json MATCHES ">>>>>>>")
    message(WARNING
      "automanip: ${_version_file} has merge conflict markers; "
      "falling back to defaults (configure will rewrite a clean file)")
    set(_json "{
  \"version\": \"0.2.0\",
  \"major\": 0,
  \"minor\": 2,
  \"patch\": 0,
  \"name\": \"Automanip\",
  \"description\": \"Manipulator HAL and OCS2 kinematic arm controller\"
}
")
  endif()
  string(JSON _name ERROR_VARIABLE _json_err GET "${_json}" name)
  if(_json_err)
    message(WARNING
      "automanip: failed to parse ${_version_file}: ${_json_err}; "
      "using defaults")
    set(_name "Automanip")
    set(_desc "Manipulator HAL and OCS2 kinematic arm controller")
    set(_major "0")
    set(_minor "2")
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
  set(_git_author "unknown")
  set(_git_email "unknown")
  set(_git_dirty "false")

  find_package(Git QUIET)
  set(_git_workdir "${_root}")
  set(_found_git_dir FALSE)
  while(NOT _found_git_dir)
    if(EXISTS "${_git_workdir}/.git")
      set(_found_git_dir TRUE)
      break()
    endif()
    get_filename_component(_parent "${_git_workdir}" DIRECTORY)
    if(_parent STREQUAL _git_workdir)
      break()
    endif()
    set(_git_workdir "${_parent}")
  endwhile()

  if(Git_FOUND AND _found_git_dir)
    # Prefer package-scoped tags: automanip-v0.1.0
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty
              --match "automanip-v*"
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _desc_pkg
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _desc_any
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_desc_pkg MATCHES "^automanip-v")
      set(_git_describe "${_desc_pkg}")
    elseif(_desc_any)
      set(_git_describe "${_desc_any}")
    endif()

    execute_process(
      COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _git_commit
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _git_branch
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    # Author date with time (git has no ms); author + email.
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" log -1
              "--date=format:%Y-%m-%d %H:%M:%S"
              --format=%cd
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _git_commit_date
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" log -1 --format=%an
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _git_author
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" log -1 --format=%ae
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _git_email
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" status --porcelain
      WORKING_DIRECTORY "${_git_workdir}"
      OUTPUT_VARIABLE _porcelain
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_porcelain)
      set(_git_dirty "true")
    endif()

    # Sync semver from describe so CLI version matches git tag:
    #   automanip-v1.2.3[-N-gSHA][-dirty]  or  v1.2.3[-N-gSHA][-dirty]
    if(_git_describe MATCHES "^automanip-v([0-9]+)\\.([0-9]+)\\.([0-9]+)")
      set(_major "${CMAKE_MATCH_1}")
      set(_minor "${CMAKE_MATCH_2}")
      set(_patch "${CMAKE_MATCH_3}")
    elseif(_git_describe MATCHES "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)")
      set(_major "${CMAKE_MATCH_1}")
      set(_minor "${CMAKE_MATCH_2}")
      set(_patch "${CMAKE_MATCH_3}")
    endif()

    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
      "${_git_workdir}/.git/index")
  else()
    message(STATUS "automanip: Git not available; using version.json only")
  endif()

  set(_version "${_major}.${_minor}.${_patch}")
  set(_full "${_version}")
  if(NOT _git_commit STREQUAL "unknown" AND NOT _git_commit STREQUAL "")
    set(_full "${_version}+g${_git_commit}")
    if(_git_dirty STREQUAL "true")
      set(_full "${_full}-dirty")
    endif()
  elseif(NOT _git_describe STREQUAL "unknown")
    set(_full "${_version}+${_git_describe}")
  endif()

  # Local wall clock: "YYYY-MM-DD" and "YYYY-MM-DD HH:MM:SS.mmm"
  string(TIMESTAMP _build_date "%Y-%m-%d")
  set(_build_time "")
  execute_process(
    COMMAND date "+%Y-%m-%d %H:%M:%S.%3N"
    OUTPUT_VARIABLE _build_time
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT _build_time)
    string(TIMESTAMP _build_time "%Y-%m-%d %H:%M:%S.000")
  endif()

  # Escape for JSON / C++ string literals.
  foreach(_esc_var IN ITEMS _git_author _git_email _name _desc)
    string(REPLACE "\\" "\\\\" ${_esc_var} "${${_esc_var}}")
    string(REPLACE "\"" "\\\"" ${_esc_var} "${${_esc_var}}")
  endforeach()

  # Stable JSON (source of truth after configure).
  set(_json_out "{
  \"version\": \"${_version}\",
  \"major\": ${_major},
  \"minor\": ${_minor},
  \"patch\": ${_patch},
  \"name\": \"${_name}\",
  \"description\": \"${_desc}\",
  \"full_version\": \"${_full}\",
  \"git_describe\": \"${_git_describe}\",
  \"git_commit\": \"${_git_commit}\",
  \"git_branch\": \"${_git_branch}\",
  \"git_commit_date\": \"${_git_commit_date}\",
  \"git_author\": \"${_git_author}\",
  \"git_email\": \"${_git_email}\",
  \"git_dirty\": ${_git_dirty},
  \"build_date\": \"${_build_date}\",
  \"build_time\": \"${_build_time}\"
}
")

  set(_gen_dir "${AUTOMANIP_GENERATED_DIR}")
  file(MAKE_DIRECTORY "${_gen_dir}")
  set(_gen_json "${_gen_dir}/version.json")
  file(WRITE "${_gen_json}" "${_json_out}")

  # Update source tree version.json only when content changes (avoid churn).
  set(_need_write TRUE)
  if(EXISTS "${_version_file}")
    file(READ "${_version_file}" _old)
    if(_old STREQUAL _json_out)
      set(_need_write FALSE)
    endif()
  endif()
  if(_need_write)
    file(WRITE "${_version_file}" "${_json_out}")
    message(STATUS "automanip: updated ${_version_file}")
  endif()

  set(AUTOMANIP_VERSION "${_version}" PARENT_SCOPE)
  set(AUTOMANIP_VERSION_MAJOR "${_major}" PARENT_SCOPE)
  set(AUTOMANIP_VERSION_MINOR "${_minor}" PARENT_SCOPE)
  set(AUTOMANIP_VERSION_PATCH "${_patch}" PARENT_SCOPE)
  set(AUTOMANIP_SOVERSION "${_major}.${_minor}" PARENT_SCOPE)
  set(AUTOMANIP_PACKAGE_NAME "${_name}" PARENT_SCOPE)
  set(AUTOMANIP_PACKAGE_DESCRIPTION "${_desc}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_DESCRIBE "${_git_describe}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_COMMIT "${_git_commit}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_BRANCH "${_git_branch}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_COMMIT_DATE "${_git_commit_date}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_AUTHOR "${_git_author}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_EMAIL "${_git_email}" PARENT_SCOPE)
  set(AUTOMANIP_GIT_DIRTY "${_git_dirty}" PARENT_SCOPE)
  set(AUTOMANIP_FULL_VERSION "${_full}" PARENT_SCOPE)
  set(AUTOMANIP_BUILD_DATE "${_build_date}" PARENT_SCOPE)
  set(AUTOMANIP_BUILD_TIME "${_build_time}" PARENT_SCOPE)

  message(STATUS
    "automanip version: ${_full} (branch ${_git_branch}, "
    "commit ${_git_commit} by ${_git_author} <${_git_email}> "
    "@ ${_git_commit_date}, built ${_build_time})")
endfunction()
