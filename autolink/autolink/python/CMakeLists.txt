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

cmake_minimum_required(VERSION 3.14)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(AUTOLINK_PYTHON_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(AUTOLINK_PYTHON_INTERNAL_BUILD_DIR "${CMAKE_BINARY_DIR}/autolink/python/internal")
file(MAKE_DIRECTORY "${AUTOLINK_PYTHON_INTERNAL_BUILD_DIR}")

# 源码里使用了 "autolink/python/internal/xxx.hpp" 头路径；
# CMake 构建时把 python/src/*.hpp 镜像到 build/autolink/python/internal。
file(GLOB AUTOLINK_PYTHON_HDRS "${AUTOLINK_PYTHON_SRC_DIR}/*.hpp")
foreach(HDR ${AUTOLINK_PYTHON_HDRS})
  get_filename_component(HDR_NAME "${HDR}" NAME)
  configure_file("${HDR}" "${AUTOLINK_PYTHON_INTERNAL_BUILD_DIR}/${HDR_NAME}" COPYONLY)
endforeach()

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '')"
  OUTPUT_VARIABLE AUTOLINK_PYTHON_EXT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE)

function(add_autolink_python_module target_name output_name source_file)
  add_library(${target_name} MODULE "${AUTOLINK_PYTHON_SRC_DIR}/${source_file}")

  target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_BINARY_DIR}
    ${Python3_INCLUDE_DIRS})

  target_link_libraries(${target_name} PRIVATE autolink Python3::Python)

  set_target_properties(${target_name} PROPERTIES
    PREFIX ""
    OUTPUT_NAME "${output_name}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/autolink/python/internal")

  if(AUTOLINK_PYTHON_EXT_SUFFIX)
    set_target_properties(${target_name} PROPERTIES SUFFIX "${AUTOLINK_PYTHON_EXT_SUFFIX}")
  endif()
endfunction()

add_autolink_python_module(autolink_py_wrapper _autolink_wrapper py_autolink.cpp)
add_autolink_python_module(autolink_py_parameter_wrapper _autolink_parameter_wrapper py_parameter.cpp)
add_autolink_python_module(autolink_py_record_wrapper _autolink_record_wrapper py_record.cpp)
add_autolink_python_module(autolink_py_time_wrapper _autolink_time_wrapper py_time.cpp)
add_autolink_python_module(autolink_py_timer_wrapper _autolink_timer_wrapper py_timer.cpp)

install(TARGETS
  autolink_py_wrapper
  autolink_py_parameter_wrapper
  autolink_py_record_wrapper
  autolink_py_time_wrapper
  autolink_py_timer_wrapper
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/autolink/python/internal)

install(FILES
  "${AUTOLINK_PYTHON_SRC_DIR}/_loader.py"
  "${AUTOLINK_PYTHON_SRC_DIR}/__init__.py"
  "${AUTOLINK_PYTHON_SRC_DIR}/autolink.py"
  "${AUTOLINK_PYTHON_SRC_DIR}/parameter.py"
  "${AUTOLINK_PYTHON_SRC_DIR}/record.py"
  "${AUTOLINK_PYTHON_SRC_DIR}/time.py"
  "${AUTOLINK_PYTHON_SRC_DIR}/timer.py"
  DESTINATION "python/autolink_py3")

if(AUTOLINK_BUILD_TEST)
  add_test(
    NAME autolink_python_smoke_import
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke_import_test.py")
  set_tests_properties(autolink_python_smoke_import PROPERTIES
    ENVIRONMENT "PYTHONPATH=${AUTOLINK_PYTHON_SRC_DIR}:${CMAKE_BINARY_DIR}/lib/autolink/python/internal")
endif()
