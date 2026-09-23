cmake_minimum_required(VERSION 3.20)

if(NOT AUTOCMAKE_SOURCE_DIR OR NOT AUTOCMAKE_BINARY_DIR OR NOT TEST_PREFIX OR NOT AUTOCMAKE_GENERATOR)
  message(FATAL_ERROR "run_version_depend.cmake: missing -D paths")
endif()

set(_src "${AUTOCMAKE_BINARY_DIR}/version-src")
set(_build "${AUTOCMAKE_BINARY_DIR}/version-build")
set(_low_src "${AUTOCMAKE_BINARY_DIR}/version-low-src")
set(_low_build "${AUTOCMAKE_BINARY_DIR}/version-low-build")
file(REMOVE_RECURSE "${TEST_PREFIX}" "${_src}" "${_build}" "${_low_src}" "${_low_build}")
file(MAKE_DIRECTORY "${_src}" "${_low_src}")

file(WRITE "${_src}/package.xml"
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>versions</name>
  <version>0.1.0</version>
  <description>Versioned dependencies.</description>
  <maintainer email=\"dev@example.com\">autocmake</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>autocmake</buildtool_depend>
  <depend version_gte=\"3.0.0\">Python3</depend>
  <exec_depend version_eq=\"2.0.0\">ExactExport</exec_depend>
  <export><build_type>autocmake</build_type></export>
</package>
")
file(WRITE "${_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(versions VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)
autocmake_project()
autocmake_build(QUIT)
autocmake_library(versions INTERFACE)
autocmake_package()
")
file(WRITE "${_low_src}/package.xml"
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>versions_low</name>
  <version>0.1.0</version>
  <description>Version upper bound.</description>
  <maintainer email=\"dev@example.com\">autocmake</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>autocmake</buildtool_depend>
  <build_depend version_lt=\"3.0.0\">Python3</build_depend>
  <export><build_type>autocmake</build_type></export>
</package>
")
file(WRITE "${_low_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(versions_low VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)
autocmake_project()
autocmake_build(QUIT)
autocmake_package()
")

execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${AUTOCMAKE_BINARY_DIR}" --prefix "${TEST_PREFIX}"
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND ${CMAKE_COMMAND}
    -G "${AUTOCMAKE_GENERATOR}"
    -S "${_src}"
    -B "${_build}"
    -DCMAKE_PREFIX_PATH=${TEST_PREFIX}
    -DCMAKE_INSTALL_PREFIX=${TEST_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${_build}" --prefix "${TEST_PREFIX}"
  COMMAND_ERROR_IS_FATAL ANY)

file(READ "${TEST_PREFIX}/lib/cmake/versions/versions-config.cmake" _config)
if(NOT _config MATCHES "find_dependency\\(Python3 3\\.0\\.0\\)")
  message(FATAL_ERROR "versions config did not export the minimum Python3 version")
endif()
if(NOT _config MATCHES "find_dependency\\(ExactExport 2\\.0\\.0 EXACT\\)")
  message(FATAL_ERROR "versions config did not export the exact dependency")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND}
    -G "${AUTOCMAKE_GENERATOR}"
    -S "${_low_src}"
    -B "${_low_build}"
    -DCMAKE_PREFIX_PATH=${TEST_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
  RESULT_VARIABLE _rc
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err)
if(_rc EQUAL 0)
  message(FATAL_ERROR "Python3 < 3.0.0 was accepted")
endif()
if(NOT _err MATCHES "does not satisfy < 3\\.0\\.0" AND NOT _out MATCHES "does not satisfy < 3\\.0\\.0")
  message(FATAL_ERROR "version failure did not report the constraint:\n${_out}\n${_err}")
endif()
