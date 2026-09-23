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

# @brief Add a docs target when Doxygen and a Doxyfile are available.
#
# @param DOXYFILE Path to a Doxyfile. Default: ${PROJECT_SOURCE_DIR}/Doxyfile.
function(autocmake_documentation)
  cmake_parse_arguments(ARG "" "DOXYFILE" "" ${ARGN})
  find_package(Doxygen QUIET)
  if(NOT DOXYGEN_FOUND)
    message(STATUS "autocmake_documentation: Doxygen was not found")
    return()
  endif()
  if(NOT ARG_DOXYFILE)
    if(EXISTS "${PROJECT_SOURCE_DIR}/Doxyfile")
      set(ARG_DOXYFILE "${PROJECT_SOURCE_DIR}/Doxyfile")
    else()
      message(STATUS "autocmake_documentation: no Doxyfile, skipping")
      return()
    endif()
  endif()
  set(_autocmake_doxygen_warn "${PROJECT_BINARY_DIR}/autocmake-doxygen.warn")
  add_custom_target(docs
    COMMAND ${CMAKE_COMMAND} -E rm -f "${_autocmake_doxygen_warn}"
    COMMAND sh -c "cat \"${ARG_DOXYFILE}\"; echo \"WARN_LOGFILE = autocmake-doxygen.warn\"" | "${DOXYGEN_EXECUTABLE}" -
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    COMMENT "Generating API documentation"
    VERBATIM)
  add_custom_target(doc_check
    COMMAND sh "${AUTOCMAKE_DIR}/tools/doc_check.sh" "${_autocmake_doxygen_warn}"
    DEPENDS docs
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    COMMENT "Checking Doxygen warnings"
    VERBATIM)
endfunction()
