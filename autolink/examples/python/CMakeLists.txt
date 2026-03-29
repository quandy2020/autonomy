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

find_package(Python3 QUIET COMPONENTS Interpreter)

file(GLOB AUTOLINK_PYTHON_EXAMPLE_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/py_*.py")

if(Python3_Interpreter_FOUND)
  add_custom_target(autolink_examples_python_syntax_check
    COMMAND ${Python3_EXECUTABLE} -m py_compile ${AUTOLINK_PYTHON_EXAMPLE_SCRIPTS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Syntax check autolink Python examples"
    VERBATIM)
endif()

if(AUTOLINK_PYTHON_EXAMPLE_SCRIPTS)
  install(PROGRAMS ${AUTOLINK_PYTHON_EXAMPLE_SCRIPTS}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/autolink/examples/python)
endif()
