cmake_minimum_required(VERSION 3.20)

if(NOT AUTOCMAKE_SOURCE_DIR OR NOT AUTOCMAKE_BINARY_DIR OR NOT TEST_PREFIX OR NOT AUTOCMAKE_GENERATOR)
  message(FATAL_ERROR "run_empty_package.cmake: missing -D paths")
endif()

set(_src "${AUTOCMAKE_BINARY_DIR}/empty-src")
set(_build "${AUTOCMAKE_BINARY_DIR}/empty-build")
set(_use_src "${AUTOCMAKE_BINARY_DIR}/empty-use-src")
set(_use_build "${AUTOCMAKE_BINARY_DIR}/empty-use-build")
file(REMOVE_RECURSE "${TEST_PREFIX}" "${_src}" "${_build}" "${_use_src}" "${_use_build}")

execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${AUTOCMAKE_BINARY_DIR}" --prefix "${TEST_PREFIX}"
  COMMAND_ERROR_IS_FATAL ANY)

file(MAKE_DIRECTORY "${_src}")
file(WRITE "${_src}/package.xml"
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>empty_pkg</name>
  <version>0.1.0</version>
  <description>Package with no compiled targets.</description>
  <maintainer email=\"dev@example.com\">autocmake</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>autocmake</buildtool_depend>
  <export><build_type>autocmake</build_type></export>
</package>
")
file(WRITE "${_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(empty_pkg VERSION 0.1.0 LANGUAGES NONE)
find_package(autocmake CONFIG REQUIRED)
autocmake_project()
autocmake_build(QUIT)
autocmake_package()
")
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
if(EXISTS "${TEST_PREFIX}/lib/cmake/empty_pkg/empty_pkgTargets.cmake")
  message(FATAL_ERROR "a package with no targets installed an export file")
endif()
if(NOT EXISTS "${TEST_PREFIX}/lib/cmake/empty_pkg/empty_pkg-config.cmake")
  message(FATAL_ERROR "empty_pkg-config.cmake was not installed")
endif()

file(MAKE_DIRECTORY "${_use_src}")
file(WRITE "${_use_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(empty_use LANGUAGES NONE)
find_package(empty_pkg CONFIG REQUIRED)
")
execute_process(
  COMMAND ${CMAKE_COMMAND}
    -G "${AUTOCMAKE_GENERATOR}"
    -S "${_use_src}"
    -B "${_use_build}"
    -DCMAKE_PREFIX_PATH=${TEST_PREFIX}
  COMMAND_ERROR_IS_FATAL ANY)
