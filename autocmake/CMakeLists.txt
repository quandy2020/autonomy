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

cmake_minimum_required(VERSION 3.20)

project(autocmake VERSION 0.1.0)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

set(AUTOCMAKE_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/AutocmakeConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/for-install/autocmake-config.cmake"
  INSTALL_DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}"
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/AutocmakeConfigVersion.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/autocmake-config-version.cmake"
  @ONLY)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/for-install/autocmake-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/autocmake-config-version.cmake"
  DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}")

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/Autocmake.cmake"
  DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}")

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cmake/"
  DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}/cmake"
  FILES_MATCHING
    PATTERN "*.cmake"
    PATTERN "*.cmake.in"
    PATTERN "*.py")

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/codecheck/"
  DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}/codecheck"
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.rule")

install(
  PROGRAMS
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/print_package_xml_version.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/check_test_ran.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/doc_check.sh"
  DESTINATION "${AUTOCMAKE_CONFIG_INSTALL_DIR}/tools")

install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/autocmake"
  DESTINATION "${CMAKE_INSTALL_BINDIR}")

# Build tree: -Dautocmake_DIR=<build> works before install.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/autocmake-config.cmake"
"set(AUTOCMAKE_DIR \"${CMAKE_CURRENT_SOURCE_DIR}\")
if(NOT AUTOCMAKE_DIR IN_LIST CMAKE_MODULE_PATH)
  list(APPEND CMAKE_MODULE_PATH \"\${AUTOCMAKE_DIR}\")
endif()
include(\"\${AUTOCMAKE_DIR}/Autocmake.cmake\")
set(autocmake_FOUND TRUE)
set(AUTOCMAKE_VERSION \"${PROJECT_VERSION}\")
")

message(STATUS "autocmake ${PROJECT_VERSION}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

if(BUILD_TESTING)
  add_test(
    NAME autocmake_hello
    COMMAND
      ${CMAKE_COMMAND}
      -DAUTOCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DAUTOCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
      -DTEST_PREFIX=${CMAKE_BINARY_DIR}/test-prefix
      -DAUTOCMAKE_GENERATOR=${CMAKE_GENERATOR}
      -P ${CMAKE_SOURCE_DIR}/tests/run_hello_example.cmake)
  set_tests_properties(autocmake_hello PROPERTIES TIMEOUT 600)

  add_test(
    NAME autocmake_workspace
    COMMAND
      ${CMAKE_COMMAND}
      -DAUTOCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DAUTOCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
      -DTEST_PREFIX=${CMAKE_BINARY_DIR}/ws-autocmake
      -P ${CMAKE_SOURCE_DIR}/tests/run_workspace.cmake)
  set_tests_properties(autocmake_workspace PROPERTIES TIMEOUT 600)

  find_package(Protobuf QUIET)
  if(Protobuf_FOUND OR Protobuf_PROTOC_EXECUTABLE)
    add_test(
      NAME autocmake_protobuf
      COMMAND
        ${CMAKE_COMMAND}
        -DAUTOCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DAUTOCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
        -DTEST_PREFIX=${CMAKE_BINARY_DIR}/proto-prefix
        -DAUTOCMAKE_GENERATOR=${CMAKE_GENERATOR}
        -P ${CMAKE_SOURCE_DIR}/tests/run_protobuf_example.cmake)
    set_tests_properties(autocmake_protobuf PROPERTIES TIMEOUT 600)
  endif()

  add_test(
    NAME autocmake_empty
    COMMAND
      ${CMAKE_COMMAND}
      -DAUTOCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DAUTOCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
      -DTEST_PREFIX=${CMAKE_BINARY_DIR}/empty-prefix
      -DAUTOCMAKE_GENERATOR=${CMAKE_GENERATOR}
      -P ${CMAKE_SOURCE_DIR}/tests/run_empty_package.cmake)
  set_tests_properties(autocmake_empty PROPERTIES TIMEOUT 600)

  add_test(
    NAME autocmake_superbuild
    COMMAND
      ${CMAKE_COMMAND}
      -DAUTOCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DAUTOCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
      -DTEST_PREFIX=${CMAKE_BINARY_DIR}/super-prefix
      -DAUTOCMAKE_GENERATOR=${CMAKE_GENERATOR}
      -P ${CMAKE_SOURCE_DIR}/tests/run_superbuild.cmake)
  set_tests_properties(autocmake_superbuild PROPERTIES TIMEOUT 600)

  add_test(
    NAME autocmake_version_depend
    COMMAND
      ${CMAKE_COMMAND}
      -DAUTOCMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DAUTOCMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
      -DTEST_PREFIX=${CMAKE_BINARY_DIR}/version-prefix
      -DAUTOCMAKE_GENERATOR=${CMAKE_GENERATOR}
      -P ${CMAKE_SOURCE_DIR}/tests/run_version_depend.cmake)
  set_tests_properties(autocmake_version_depend PROPERTIES TIMEOUT 600)

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_EXECUTABLE)
    add_test(
      NAME autocmake_package_xml
      COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_SOURCE_DIR}/tests/test_package_xml.py"
        "${CMAKE_SOURCE_DIR}/cmake/package_xml.py")
  endif()
endif()
