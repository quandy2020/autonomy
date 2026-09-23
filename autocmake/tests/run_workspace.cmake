cmake_minimum_required(VERSION 3.20)

if(NOT AUTOCMAKE_SOURCE_DIR OR NOT AUTOCMAKE_BINARY_DIR OR NOT TEST_PREFIX)
  message(FATAL_ERROR "run_workspace.cmake: missing -D paths")
endif()

find_program(Python3_EXECUTABLE python3 REQUIRED)

set(_autocmake "${AUTOCMAKE_SOURCE_DIR}/scripts/autocmake")
set(_ws "${AUTOCMAKE_SOURCE_DIR}/example/workspace")
set(_build "${AUTOCMAKE_BINARY_DIR}/ws-build")
set(_install "${AUTOCMAKE_BINARY_DIR}/ws-install")

file(REMOVE_RECURSE "${TEST_PREFIX}" "${_build}" "${_install}")

execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${AUTOCMAKE_BINARY_DIR}" --prefix "${TEST_PREFIX}"
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${Python3_EXECUTABLE} ${_autocmake} list --base-path ${_ws}
  OUTPUT_VARIABLE _order
  COMMAND_ERROR_IS_FATAL ANY)
string(FIND "${_order}" "base" _base_at)
string(FIND "${_order}" "app" _app_at)
if(_base_at LESS 0 OR _app_at LESS 0 OR _base_at GREATER _app_at)
  message(FATAL_ERROR "expected base before app, got: ${_order}")
endif()

set(ENV{CMAKE_PREFIX_PATH} "${TEST_PREFIX}")
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${_autocmake} build
    --base-path ${_ws}
    --build-base ${_build}
    --install-base ${_install}
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${Python3_EXECUTABLE} ${_autocmake} test
    --base-path ${_ws}
    --build-base ${_build}
    --install-base ${_install}
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${Python3_EXECUTABLE} ${_autocmake} index --prefix ${_install}
  OUTPUT_VARIABLE _indexed
  COMMAND_ERROR_IS_FATAL ANY)
string(FIND "${_indexed}" "base" _has_base)
string(FIND "${_indexed}" "app" _has_app)
if(_has_base LESS 0 OR _has_app LESS 0)
  message(FATAL_ERROR "index missing packages: ${_indexed}")
endif()

if(NOT EXISTS "${_install}/share/base/environment/library_path.dsv")
  message(FATAL_ERROR "environment hook was not installed")
endif()
execute_process(
  COMMAND bash -c "source \"${_install}/share/base/environment/library_path.sh\" && printf %s \"\$PATH\""
  OUTPUT_VARIABLE _hook_path
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT _hook_path MATCHES "^${_install}/bin")
  message(FATAL_ERROR "environment hook did not prepend bin: ${_hook_path}")
endif()

execute_process(
  COMMAND ${_install}/bin/app_main
  OUTPUT_VARIABLE _app_out
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT _app_out STREQUAL "3\n")
  message(FATAL_ERROR "app_main printed '${_app_out}', expected 3")
endif()
