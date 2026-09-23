cmake_minimum_required(VERSION 3.20)

if(NOT AUTOCMAKE_SOURCE_DIR OR NOT AUTOCMAKE_BINARY_DIR OR NOT TEST_PREFIX OR NOT AUTOCMAKE_GENERATOR)
  message(FATAL_ERROR "run_hello_example.cmake: missing -D AUTOCMAKE_SOURCE_DIR, AUTOCMAKE_BINARY_DIR, TEST_PREFIX, or AUTOCMAKE_GENERATOR")
endif()

set(_hello_src "${AUTOCMAKE_SOURCE_DIR}/example/hello")
set(_hello_build "${AUTOCMAKE_BINARY_DIR}/hello-build")
set(_consumer_src "${AUTOCMAKE_BINARY_DIR}/consumer-src")
set(_consumer_build "${AUTOCMAKE_BINARY_DIR}/consumer-build")

file(REMOVE_RECURSE "${TEST_PREFIX}" "${_hello_build}" "${_consumer_src}" "${_consumer_build}")

execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${AUTOCMAKE_BINARY_DIR}" --prefix "${TEST_PREFIX}"
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${CMAKE_COMMAND}
    -G "${AUTOCMAKE_GENERATOR}"
    -S "${_hello_src}"
    -B "${_hello_build}"
    -DCMAKE_PREFIX_PATH=${TEST_PREFIX}
    -DCMAKE_INSTALL_PREFIX=${TEST_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_TESTING=ON
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${CMAKE_COMMAND} --build "${_hello_build}" --config Release --parallel
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${CMAKE_CTEST_COMMAND} --test-dir "${_hello_build}" --build-config Release --output-on-failure
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

set(_hello_main "")
foreach(_candidate
    "${_hello_build}/hello_main"
    "${_hello_build}/Release/hello_main"
    "${_hello_build}/RelWithDebInfo/hello_main")
  if(EXISTS "${_candidate}")
    set(_hello_main "${_candidate}")
  endif()
endforeach()
if(NOT _hello_main)
  message(FATAL_ERROR "hello_main was not built")
endif()
execute_process(COMMAND "${_hello_main}" RESULT_VARIABLE _rc COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${_hello_build}" --prefix "${TEST_PREFIX}" --config Release
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

if(NOT EXISTS "${TEST_PREFIX}/share/autocmake_index/resource_index/packages/hello")
  message(FATAL_ERROR "autocmake index is missing the hello package marker")
endif()
if(NOT EXISTS "${TEST_PREFIX}/share/hello/package.xml")
  message(FATAL_ERROR "package.xml was not installed")
endif()
file(READ "${TEST_PREFIX}/share/hello/environment/library_path.sh" _hook)
if(NOT _hook MATCHES "PYTHONPATH=\"\\$\\{_autocmake_prefix\\}/lib/python")
  message(FATAL_ERROR "library_path.sh does not prepend lib/python to PYTHONPATH")
endif()
if(APPLE)
  execute_process(
    COMMAND otool -l "${TEST_PREFIX}/bin/hello_main"
    OUTPUT_VARIABLE _rpath
    COMMAND_ERROR_IS_FATAL ANY)
  if(NOT _rpath MATCHES "@loader_path/\\.\\./lib")
    message(FATAL_ERROR "hello_main RPATH does not point at the install lib directory")
  endif()
endif()

file(MAKE_DIRECTORY "${_consumer_src}")
file(WRITE "${_consumer_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(consumer)
find_package(hello CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE hello::core hello::greet)
")
file(WRITE "${_consumer_src}/main.cpp"
"#include \"hello/greet/greet.hpp\"
#include \"hello/hello.hpp\"
#include <string>
int main() {
  return (hello_value() == 7 && greet() == std::string(\"hello\")) ? 0 : 1;
}
")

execute_process(
  COMMAND ${CMAKE_COMMAND}
    -G "${AUTOCMAKE_GENERATOR}"
    -S "${_consumer_src}"
    -B "${_consumer_build}"
    -DCMAKE_PREFIX_PATH=${TEST_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

execute_process(
  COMMAND ${CMAKE_COMMAND} --build "${_consumer_build}" --config Release
  RESULT_VARIABLE _rc
  COMMAND_ERROR_IS_FATAL ANY)

set(_consumer "")
foreach(_candidate
    "${_consumer_build}/consumer"
    "${_consumer_build}/Release/consumer")
  if(EXISTS "${_candidate}")
    set(_consumer "${_candidate}")
  endif()
endforeach()
if(NOT _consumer)
  message(FATAL_ERROR "consumer was not built")
endif()
execute_process(COMMAND "${_consumer}" RESULT_VARIABLE _rc COMMAND_ERROR_IS_FATAL ANY)
