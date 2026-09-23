cmake_minimum_required(VERSION 3.20)

if(NOT AUTOCMAKE_SOURCE_DIR OR NOT AUTOCMAKE_BINARY_DIR OR NOT TEST_PREFIX OR NOT AUTOCMAKE_GENERATOR)
  message(FATAL_ERROR "run_protobuf_example.cmake: missing -D paths")
endif()

set(_src "${AUTOCMAKE_SOURCE_DIR}/example/messages")
set(_build "${AUTOCMAKE_BINARY_DIR}/messages-build")

file(REMOVE_RECURSE "${TEST_PREFIX}" "${_build}")

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
  COMMAND ${CMAKE_COMMAND} --build "${_build}" --config Release --parallel
  COMMAND_ERROR_IS_FATAL ANY)

set(_main "")
foreach(_candidate
    "${_build}/messages_main"
    "${_build}/Release/messages_main"
    "${_build}/RelWithDebInfo/messages_main")
  if(EXISTS "${_candidate}")
    set(_main "${_candidate}")
  endif()
endforeach()
if(NOT _main)
  message(FATAL_ERROR "messages_main was not built")
endif()
execute_process(
  COMMAND "${_main}"
  OUTPUT_VARIABLE _out
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT _out STREQUAL "ok 3\n")
  message(FATAL_ERROR "messages_main printed '${_out}', expected 'ok 3'")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${_build}" --prefix "${TEST_PREFIX}" --config Release
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT EXISTS "${TEST_PREFIX}/share/messages/proto/greeting.proto")
  message(FATAL_ERROR "greeting.proto was not installed")
endif()
if(NOT EXISTS "${TEST_PREFIX}/include/proto/greeting.pb.h")
  message(FATAL_ERROR "greeting.pb.h was not installed")
endif()
if(NOT EXISTS "${TEST_PREFIX}/lib/python/proto/greeting_pb2.py")
  message(FATAL_ERROR "greeting_pb2.py was not installed")
endif()

find_package(Python3 COMPONENTS Interpreter REQUIRED)
file(READ "${TEST_PREFIX}/lib/python/proto/greeting_pb2.py" _generated LIMIT 500)
if(NOT _generated MATCHES "Protobuf Python Version: ([0-9.]+)")
  message(FATAL_ERROR "generated Python has no Protobuf Python Version")
endif()
set(_runtime_version "${CMAKE_MATCH_1}")
set(_venv "${AUTOCMAKE_BINARY_DIR}/protobuf-python-venv")
set(_install_runtime TRUE)
if(EXISTS "${_venv}/bin/python")
  execute_process(
    COMMAND "${_venv}/bin/python" -c "import google.protobuf; print(google.protobuf.__version__)"
    OUTPUT_VARIABLE _installed_runtime
    RESULT_VARIABLE _runtime_rc)
  if(_runtime_rc EQUAL 0 AND _installed_runtime STREQUAL "${_runtime_version}\n")
    set(_install_runtime FALSE)
  endif()
endif()
if(_install_runtime)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m venv "${_venv}"
    COMMAND_ERROR_IS_FATAL ANY)
  execute_process(
    COMMAND "${_venv}/bin/python" -m pip install "protobuf==${_runtime_version}"
    COMMAND_ERROR_IS_FATAL ANY)
endif()
execute_process(
  COMMAND "${_venv}/bin/python" -c
    "import sys; sys.path.insert(0, sys.argv[1]); from proto import greeting_pb2; greeting = greeting_pb2.Greeting(); greeting.text = 'ok'; greeting.value = 3; greeting.note.text = 'proto3'; assert greeting.note.text == 'proto3'; print('%s %s' % (greeting.text, greeting.value))"
    "${TEST_PREFIX}/lib/python"
  OUTPUT_VARIABLE _python_out
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT _python_out STREQUAL "ok 3\n")
  message(FATAL_ERROR "python greeting printed '${_python_out}', expected 'ok 3'")
endif()

set(_reader_src "${AUTOCMAKE_BINARY_DIR}/reader-src")
set(_reader_build "${AUTOCMAKE_BINARY_DIR}/reader-build")
file(REMOVE_RECURSE "${_reader_src}" "${_reader_build}")
file(MAKE_DIRECTORY "${_reader_src}/proto" "${_reader_src}/src")
file(WRITE "${_reader_src}/proto/reader.proto"
"syntax = \"proto3\";
package reader;
import \"proto/greeting.proto\";
message Reader {
  messages.Greeting greeting = 1;
}
")
file(WRITE "${_reader_src}/src/main.cpp"
"#include \"proto/reader.pb.h\"
int main() {
  reader::Reader message;
  message.mutable_greeting()->set_text(\"ok\");
  message.mutable_greeting()->set_value(3);
  return message.greeting().value() == 3 ? 0 : 1;
}
")
file(WRITE "${_reader_src}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.20)
project(reader VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)
find_package(messages CONFIG REQUIRED)
autocmake_project()
autocmake_build(QUIT)
autocmake_protobuf(reader
  SOURCES proto/reader.proto
  IMPORTS \${CMAKE_CURRENT_SOURCE_DIR} \${messages_PROTO_PATH}
  DEPENDENCIES messages::messages)
autocmake_binary(reader_main SOURCES src/main.cpp DEPENDENCIES reader)
")
execute_process(
  COMMAND ${CMAKE_COMMAND}
    -G "${AUTOCMAKE_GENERATOR}"
    -S "${_reader_src}"
    -B "${_reader_build}"
    -DCMAKE_PREFIX_PATH=${TEST_PREFIX}
    -DCMAKE_INSTALL_PREFIX=${TEST_PREFIX}
    -DCMAKE_BUILD_TYPE=Release
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND ${CMAKE_COMMAND} --build "${_reader_build}" --config Release --parallel
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND ${CMAKE_COMMAND} --install "${_reader_build}" --prefix "${TEST_PREFIX}" --config Release
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND "${TEST_PREFIX}/bin/reader_main" COMMAND_ERROR_IS_FATAL ANY)
