cmake_minimum_required(VERSION 3.20)

if(NOT AUTOCMAKE_SOURCE_DIR OR NOT AUTOCMAKE_BINARY_DIR OR NOT TEST_PREFIX OR NOT AUTOCMAKE_GENERATOR)
  message(FATAL_ERROR "run_superbuild.cmake: missing -D paths")
endif()

set(_src "${AUTOCMAKE_BINARY_DIR}/super-src")
set(_build "${AUTOCMAKE_BINARY_DIR}/super-build")
file(REMOVE_RECURSE "${TEST_PREFIX}" "${_src}" "${_build}")
file(MAKE_DIRECTORY "${_src}/guest")

file(WRITE "${_src}/package.xml"
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>host</name>
  <version>0.1.0</version>
  <description>Parent package.</description>
  <maintainer email=\"dev@example.com\">autocmake</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>autocmake</buildtool_depend>
  <export><build_type>autocmake</build_type></export>
</package>
")
file(WRITE "${_src}/guest/package.xml"
"<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>guest</name>
  <version>0.3.0</version>
  <description>Nested package.</description>
  <maintainer email=\"dev@example.com\">autocmake</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>autocmake</buildtool_depend>
  <export><build_type>autocmake</build_type></export>
</package>
")
file(WRITE "${_src}/guest/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(guest)
find_package(autocmake CONFIG REQUIRED)
autocmake_project()
autocmake_export(HostDepMarker)
autocmake_build(QUIT)
autocmake_library(guest INTERFACE)
autocmake_package()
")
file(WRITE "${_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(host VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)
autocmake_project()
autocmake_export(GuestDepMarker)
autocmake_build(QUIT)
add_subdirectory(guest)
autocmake_library(host INTERFACE)
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

file(READ "${TEST_PREFIX}/lib/cmake/host/host-config.cmake" _host_config)
file(READ "${TEST_PREFIX}/lib/cmake/guest/guest-config.cmake" _guest_config)
file(READ "${TEST_PREFIX}/lib/cmake/guest/guest-config-version.cmake" _guest_version)
if(NOT _host_config MATCHES "find_dependency\\(GuestDepMarker\\)")
  message(FATAL_ERROR "host config lost its own exported dependency")
endif()
if(_host_config MATCHES "HostDepMarker")
  message(FATAL_ERROR "host config picked up the nested package dependency")
endif()
if(NOT _guest_config MATCHES "find_dependency\\(HostDepMarker\\)")
  message(FATAL_ERROR "guest config lost its exported dependency")
endif()
if(_guest_config MATCHES "GuestDepMarker")
  message(FATAL_ERROR "guest config picked up the parent dependency")
endif()
if(NOT _guest_version MATCHES "0\\.3\\.0")
  message(FATAL_ERROR "guest did not take its version from package.xml")
endif()
