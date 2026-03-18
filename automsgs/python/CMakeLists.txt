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

# Install package __init__.py so "import automsgs" works (generated _pb2 modules are installed by proto/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/__init__.py
  DESTINATION ${AUTOMSGS_PYTHON_INSTALL_PATH}/automsgs)

if(BUILD_TESTING AND NOT WIN32)
  set(python_tests
    basic_TEST
  )
  execute_process(COMMAND "${Python3_EXECUTABLE}" -m pytest --version
    OUTPUT_VARIABLE PYTEST_output
    ERROR_VARIABLE  PYTEST_error
    RESULT_VARIABLE PYTEST_result)
  if(PYTEST_result EQUAL 0)
    set(pytest_FOUND TRUE)
  else()
    message(WARNING "Pytest package not available: ${PYTEST_error}")
    message(WARNING "Output: ${PYTEST_output}")
  endif()

  foreach(test ${python_tests})
    if(pytest_FOUND)
      add_test(NAME ${test}.py COMMAND
        "${Python3_EXECUTABLE}" -m pytest
        "${CMAKE_SOURCE_DIR}/python/test/${test}.py"
        --junitxml "${CMAKE_BINARY_DIR}/test_results/${test}.xml")
    else()
      add_test(NAME ${test}.py COMMAND
        "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/python/test/${test}.py")
    endif()
    set(_env_vars "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${AUTOMSGS_PYTHON_INSTALL_PATH}:${CMAKE_BINARY_DIR}/proto/gen/python")
    set_tests_properties(${test}.py PROPERTIES ENVIRONMENT "${_env_vars}")
  endforeach()
endif()
