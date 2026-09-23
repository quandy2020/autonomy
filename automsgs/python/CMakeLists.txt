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

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/__init__.py"
  DESTINATION "${AUTOMSGS_PYTHON_INSTALL_PATH}/automsgs")

if(NOT BUILD_TESTING OR WIN32)
  return()
endif()

execute_process(COMMAND "${Python3_EXECUTABLE}" -m pytest --version
  RESULT_VARIABLE _pytest_ok OUTPUT_QUIET ERROR_QUIET)
file(GLOB _tests CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*_TEST.py")
foreach(_test ${_tests})
  cmake_path(GET _test STEM _name)
  if(_pytest_ok EQUAL 0)
    add_test(NAME "${_name}.py" COMMAND
      "${Python3_EXECUTABLE}" -m pytest "${_test}"
      --junitxml "${CMAKE_BINARY_DIR}/test_results/${_name}.xml")
  else()
    add_test(NAME "${_name}.py" COMMAND "${Python3_EXECUTABLE}" "${_test}")
  endif()
  set_tests_properties("${_name}.py" PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${AUTOMSGS_PYTHON_INSTALL_PATH}:${AUTOMSGS_PROTO_GEN_DIR}/python")
endforeach()
