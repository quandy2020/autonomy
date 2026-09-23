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

# @brief Find GTest, or fetch release 1.15.2 when it is not installed.
macro(_autocmake_ensure_gtest)
  if(NOT TARGET GTest::gtest_main AND NOT TARGET GTest::Main)
    find_package(GTest QUIET)
  endif()
  if(NOT TARGET GTest::gtest_main AND NOT TARGET GTest::Main)
    include(FetchContent)
    if(POLICY CMP0135)
      cmake_policy(SET CMP0135 NEW)
    endif()
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
    FetchContent_MakeAvailable(googletest)
  endif()
endmacro()

# @brief Register a gtest with CTest and a follow-up result check.
#
# The test writes gtest xml under test_results/. check_<name> runs
# tools/check_test_ran.py so a crash that produced no xml still fails.
function(_autocmake_register_test name)
  set(_result "${CMAKE_BINARY_DIR}/test_results/${name}.xml")
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/test_results")
  add_test(NAME ${name} COMMAND ${name} --gtest_output=xml:${_result})
  if(NOT Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter QUIET)
  endif()
  if(Python3_EXECUTABLE)
    add_test(
      NAME check_${name}
      COMMAND "${Python3_EXECUTABLE}"
        "${AUTOCMAKE_DIR}/tools/check_test_ran.py" "${_result}")
    set_tests_properties(check_${name} PROPERTIES DEPENDS ${name})
  endif()
endfunction()

# @brief Link a test executable to GTest's main.
function(_autocmake_link_gtest target)
  if(TARGET GTest::gtest_main)
    target_link_libraries(${target} PRIVATE GTest::gtest_main)
  elseif(TARGET GTest::Main)
    target_link_libraries(${target} PRIVATE GTest::Main GTest::GTest)
  else()
    message(FATAL_ERROR "autocmake: GTest was not found")
  endif()
endfunction()

# @brief Register one gtest executable. Tests are not installed.
#
# @param target Test target and CTest name.
# @param SOURCES Sources.
# @param DEPENDENCIES Libraries linked PRIVATE.
# @param NO_MAIN Link GTest::gtest instead of gtest_main.
function(autocmake_test target)
  if(NOT BUILD_TESTING)
    return()
  endif()
  _autocmake_parse(_arg "NO_MAIN" "" "SOURCES;DEPENDENCIES" ${ARGN})
  if(NOT _arg_SOURCES)
    message(FATAL_ERROR "autocmake_test(${target}): SOURCES is required")
  endif()
  _autocmake_ensure_gtest()
  add_executable(${target} ${_arg_SOURCES})
  if(_arg_NO_MAIN)
    if(TARGET GTest::gtest)
      target_link_libraries(${target} PRIVATE GTest::gtest)
    else()
      target_link_libraries(${target} PRIVATE GTest::GTest)
    endif()
  else()
    _autocmake_link_gtest(${target})
  endif()
  if(_arg_DEPENDENCIES)
    target_link_libraries(${target} PRIVATE ${_arg_DEPENDENCIES})
  endif()
  _autocmake_register_test(${target})
endfunction()
