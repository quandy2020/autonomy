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

# Map a package.xml test_depend onto a codecheck/check.py lint tool.
# Empty means this depend is not a linter autocmake knows.
function(_autocmake_lint_tool output depend)
  if(depend MATCHES "(^|_)clang-format$" OR depend MATCHES "(^|_)clang_format$")
    set(_tool clang-format)
  elseif(depend MATCHES "(^|_)clang-tidy$" OR depend MATCHES "(^|_)clang_tidy$")
    set(_tool clang-tidy)
  elseif(depend MATCHES "(^|_)lint_cmake$")
    set(_tool cmakelint)
  elseif(depend MATCHES "(^|_)(cppcheck|cpplint|flake8|pycodestyle|pyflakes|pep257|mypy|xmllint|uncrustify|copyright)$")
    set(_tool "${CMAKE_MATCH_2}")
  else()
    set(_tool "")
  endif()
  set(${output} "${_tool}" PARENT_SCOPE)
endfunction()

# @brief Register linters named by package.xml test_depend.
#
# Recognized names are the tool, or the ament package that ends with that tool:
# clang-format, cppcheck, cpplint, flake8, pycodestyle, pyflakes, pep257,
# mypy, xmllint, lint_cmake, uncrustify, clang-tidy, copyright.
# A missing program fails the test. Other test_depend names are ignored.
macro(autocmake_lint)
  if(BUILD_TESTING)
    _autocmake_ensure_xml()
    if(NOT Python3_EXECUTABLE)
      find_package(Python3 COMPONENTS Interpreter REQUIRED)
    endif()
    foreach(_depend IN LISTS ${PROJECT_NAME}_TEST_DEPENDS)
      _autocmake_lint_tool(_tool "${_depend}")
      if(_tool)
        add_test(
          NAME ${PROJECT_NAME}_${_tool}
          COMMAND "${Python3_EXECUTABLE}" "${AUTOCMAKE_DIR}/codecheck/check.py"
            lint ${_tool} "${PROJECT_SOURCE_DIR}")
        set_tests_properties(${PROJECT_NAME}_${_tool} PROPERTIES
          LABELS "linter;${_tool}"
          WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
      endif()
    endforeach()
  endif()
endmacro()

# @brief Add cppcheck, cpplint, and codecheck targets when cppcheck exists.
#
# codecheck depends on cppcheck. cpplint is added when that program is on PATH.
# cppcheck uses the .rule files in autocmake/codecheck when this cppcheck still
# accepts --rule-file. A cppcheck.suppress file in the source or build tree
# is passed through.
function(_autocmake_add_codecheck)
  if(TARGET codecheck)
    return()
  endif()
  find_program(_autocmake_cppcheck cppcheck)
  if(NOT _autocmake_cppcheck)
    message(STATUS "autocmake: cppcheck was not found, codecheck target was not added")
    return()
  endif()
  if(NOT Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter QUIET)
  endif()
  if(NOT Python3_EXECUTABLE)
    message(STATUS "autocmake: Python3 was not found, codecheck target was not added")
    return()
  endif()

  set(_includes "")
  if(EXISTS "${PROJECT_SOURCE_DIR}/include")
    list(APPEND _includes "${PROJECT_SOURCE_DIR}/include")
  endif()
  if(EXISTS "${PROJECT_BINARY_DIR}/include")
    list(APPEND _includes "${PROJECT_BINARY_DIR}/include")
  endif()
  set(_include_args "")
  foreach(_dir IN LISTS _includes)
    list(APPEND _include_args --include "${_dir}")
  endforeach()

  set(_check "${Python3_EXECUTABLE}" "${AUTOCMAKE_DIR}/codecheck/check.py")
  add_custom_target(cppcheck
    COMMAND ${_check} lint cppcheck "${PROJECT_SOURCE_DIR}" ${_include_args}
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    COMMENT "cppcheck"
    VERBATIM)
  add_custom_target(codecheck
    COMMAND ${_check} codecheck "${PROJECT_SOURCE_DIR}" ${_include_args}
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    COMMENT "cppcheck and cpplint"
    VERBATIM)
  find_program(_autocmake_cpplint cpplint)
  if(_autocmake_cpplint)
    add_custom_target(cpplint
      COMMAND ${_check} lint cpplint "${PROJECT_SOURCE_DIR}"
      WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
      COMMENT "cpplint"
      VERBATIM)
  endif()
endfunction()
