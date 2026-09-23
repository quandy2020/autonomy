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

include_guard(GLOBAL)

# @brief Extra protoc arguments for proto3 optional on protoc older than 3.15.
function(_autocmake_protoc_arguments out_var)
  if(_AUTOCMAKE_PROTOC_ARGUMENTS_READY)
    set(${out_var} "${_AUTOCMAKE_PROTOC_ARGUMENTS}" PARENT_SCOPE)
    return()
  endif()
  execute_process(
    COMMAND "${Protobuf_PROTOC_EXECUTABLE}" --version
    OUTPUT_VARIABLE _version
    ERROR_VARIABLE _version_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE)
  set(_version "${_version}${_version_error}")
  set(_arguments "")
  if(_version MATCHES "([0-9]+)\\.([0-9]+)")
    if(CMAKE_MATCH_1 LESS 3 OR (CMAKE_MATCH_1 EQUAL 3 AND CMAKE_MATCH_2 LESS 15))
      set(_arguments "--experimental_allow_proto3_optional")
    endif()
  endif()
  set(_AUTOCMAKE_PROTOC_ARGUMENTS "${_arguments}" CACHE INTERNAL "")
  set(_AUTOCMAKE_PROTOC_ARGUMENTS_READY TRUE CACHE INTERNAL "")
  set(${out_var} "${_arguments}" PARENT_SCOPE)
endfunction()

# @brief Find protoc and record Protobuf as a public dependency.
function(_autocmake_require_protobuf)
  if(NOT TARGET protobuf::libprotobuf OR NOT Protobuf_PROTOC_EXECUTABLE)
    find_package(Protobuf CONFIG QUIET)
  endif()
  if(NOT TARGET protobuf::libprotobuf OR NOT Protobuf_PROTOC_EXECUTABLE)
    find_package(Protobuf REQUIRED)
  endif()
  if(NOT Protobuf_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "autocmake_protobuf: protoc was not found")
  endif()
  set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_PUBLIC_DEPENDENCIES
    "find_dependency(Protobuf)")
endfunction()

# @brief Normalize IMPORTS. The first entry is the layout root.
function(_autocmake_proto_layout imports out_imports out_root)
  if(NOT imports)
    set(imports "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  set(_imports "")
  foreach(_import IN LISTS imports)
    cmake_path(ABSOLUTE_PATH _import BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" NORMALIZE)
    list(APPEND _imports "${_import}")
  endforeach()
  list(GET _imports 0 _root)
  set(${out_imports} "${_imports}" PARENT_SCOPE)
  set(${out_root} "${_root}" PARENT_SCOPE)
endfunction()

# @brief Install Python package markers for one generated module path.
function(_autocmake_proto_python_init relative_dir)
  set(_package_dir "${relative_dir}")
  while(_package_dir)
    get_property(_done DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY AUTOCMAKE_PYTHON_PACKAGES)
    if(NOT _package_dir IN_LIST _done)
      set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
        AUTOCMAKE_PYTHON_PACKAGES "${_package_dir}")
      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${_package_dir}")
      set(_init "${CMAKE_CURRENT_BINARY_DIR}/${_package_dir}/__init__.py")
      if(NOT EXISTS "${_init}")
        file(WRITE "${_init}" "")
      endif()
      install(FILES "${_init}" DESTINATION "${CMAKE_INSTALL_LIBDIR}/python/${_package_dir}")
    endif()
    cmake_path(GET _package_dir PARENT_PATH _package_dir)
  endwhile()
endfunction()

# @brief Run protoc for one proto3 file and install its outputs.
function(_autocmake_proto_one target proto layout_root imports protoc_arguments)
  cmake_path(ABSOLUTE_PATH proto BASE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" NORMALIZE)
  if(NOT EXISTS "${proto}")
    message(FATAL_ERROR "autocmake_protobuf(${target}): missing ${proto}")
  endif()
  file(READ "${proto}" _proto_text LIMIT 400)
  if(NOT _proto_text MATCHES "syntax[ \t]*=[ \t]*\"proto3\"")
    message(FATAL_ERROR "autocmake_protobuf(${target}): ${proto} is not proto3")
  endif()
  cmake_path(IS_PREFIX layout_root "${proto}" NORMALIZE _under_root)
  if(NOT _under_root)
    message(FATAL_ERROR
      "autocmake_protobuf(${target}): ${proto} is outside the layout root ${layout_root}")
  endif()
  cmake_path(RELATIVE_PATH proto BASE_DIRECTORY "${layout_root}" OUTPUT_VARIABLE _relative)
  cmake_path(REPLACE_EXTENSION _relative LAST_ONLY ".pb.cc" OUTPUT_VARIABLE _relative_source)
  cmake_path(REPLACE_EXTENSION _relative LAST_ONLY ".pb.h" OUTPUT_VARIABLE _relative_header)
  cmake_path(REMOVE_EXTENSION _relative OUTPUT_VARIABLE _relative_stem)
  set(_source "${CMAKE_CURRENT_BINARY_DIR}/${_relative_source}")
  set(_header "${CMAKE_CURRENT_BINARY_DIR}/${_relative_header}")
  set(_python "${CMAKE_CURRENT_BINARY_DIR}/${_relative_stem}_pb2.py")
  set(_proto_paths "--proto_path=${layout_root}")
  foreach(_import IN LISTS imports)
    if(NOT _import STREQUAL layout_root)
      list(APPEND _proto_paths "--proto_path=${_import}")
    endif()
  endforeach()
  add_custom_command(
    OUTPUT "${_source}" "${_header}" "${_python}"
    COMMAND "${Protobuf_PROTOC_EXECUTABLE}"
      ${protoc_arguments}
      "--cpp_out=${CMAKE_CURRENT_BINARY_DIR}"
      "--python_out=${CMAKE_CURRENT_BINARY_DIR}"
      ${_proto_paths}
      "${proto}"
    DEPENDS "${proto}"
    COMMENT "autocmake protobuf ${_relative}"
    VERBATIM)
  set_source_files_properties("${_source}" "${_header}" "${_python}" PROPERTIES GENERATED TRUE)
  cmake_path(GET _relative PARENT_PATH _relative_dir)
  install(FILES "${proto}" DESTINATION "share/${PROJECT_NAME}/${_relative_dir}")
  install(FILES "${_header}" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${_relative_dir}")
  install(FILES "${_python}" DESTINATION "${CMAKE_INSTALL_LIBDIR}/python/${_relative_dir}")
  _autocmake_proto_python_init("${_relative_dir}")
  set(_autocmake_proto_source "${_source}" PARENT_SCOPE)
  set(_autocmake_proto_header "${_header}" PARENT_SCOPE)
endfunction()

# @brief Create or extend the library and link Protobuf.
function(_autocmake_proto_attach target sources headers dependencies)
  if(NOT TARGET ${target})
    autocmake_library(${target} SOURCES ${sources} ${headers})
  else()
    target_sources(${target} PRIVATE ${sources} ${headers})
  endif()
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  if(TARGET protobuf::libprotobuf)
    target_link_libraries(${target} PUBLIC protobuf::libprotobuf)
  else()
    target_link_libraries(${target} PUBLIC ${Protobuf_LIBRARIES})
    target_include_directories(${target} PUBLIC ${Protobuf_INCLUDE_DIRS})
  endif()
  if(dependencies)
    target_link_libraries(${target} PUBLIC ${dependencies})
  endif()
endfunction()

# @brief Publish proto and Python paths through the package config.
function(_autocmake_proto_config layout_root)
  file(RELATIVE_PATH _prefix_from_config
    "/opt/${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    "/opt")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-proto.cmake"
    "set(${PROJECT_NAME}_PROTO_PATH \"\${CMAKE_CURRENT_LIST_DIR}/${_prefix_from_config}/share/${PROJECT_NAME}\")\n"
    "set(${PROJECT_NAME}_PYTHON_PATH \"\${CMAKE_CURRENT_LIST_DIR}/${_prefix_from_config}/${CMAKE_INSTALL_LIBDIR}/python\")\n")
  get_property(_proto_config_ready DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    PROPERTY AUTOCMAKE_PROTO_CONFIG_READY)
  if(NOT _proto_config_ready)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY AUTOCMAKE_CONFIG_EXTRAS
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-proto.cmake")
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      PROPERTY AUTOCMAKE_PROTO_CONFIG_READY TRUE)
  endif()
  set(${PROJECT_NAME}_PROTO_PATH "${layout_root}" PARENT_SCOPE)
  set(${PROJECT_NAME}_PYTHON_PATH "${CMAKE_CURRENT_BINARY_DIR}" PARENT_SCOPE)
endfunction()

# @brief Compile proto3 files and attach the C++ sources to a library.
#
# The first IMPORTS entry is the layout root. A file proto/a.proto under that
# root becomes ${CMAKE_CURRENT_BINARY_DIR}/proto/a.pb.h, included as
# "proto/a.pb.h". Later IMPORTS entries are extra protoc search paths, used
# for import "sibling.proto" and for another package's installed protos.
#
# Generated headers install under include/. The .proto files keep their path
# relative to the layout root, under share/${PROJECT_NAME}/. Downstream
# packages see ${PROJECT_NAME}_PROTO_PATH pointing at that directory.
# The same invocation writes <name>_pb2.py under ${CMAKE_INSTALL_LIBDIR}/python.
# Downstream packages then see ${PROJECT_NAME}_PYTHON_PATH.
#
# @param target Library that receives the generated sources. Created when absent.
# @param SOURCES Proto files. Each file must contain syntax = "proto3".
# @param IMPORTS Directories passed to protoc --proto_path.
#        Default: CMAKE_CURRENT_SOURCE_DIR.
# @param DEPENDENCIES Extra PUBLIC link dependencies.
function(autocmake_protobuf target)
  _autocmake_parse(_arg "" "" "SOURCES;IMPORTS;DEPENDENCIES" ${ARGN})
  if(NOT _arg_SOURCES)
    message(FATAL_ERROR "autocmake_protobuf(${target}): SOURCES is required")
  endif()
  if(NOT AUTOCMAKE_EXPORT_NAME)
    message(FATAL_ERROR "autocmake_protobuf: call autocmake_project() first")
  endif()

  _autocmake_require_protobuf()
  _autocmake_proto_layout("${_arg_IMPORTS}" _imports _layout_root)
  _autocmake_protoc_arguments(_protoc_arguments)
  set(_sources "")
  set(_headers "")
  foreach(_proto IN LISTS _arg_SOURCES)
    _autocmake_proto_one(
      "${target}" "${_proto}" "${_layout_root}" "${_imports}" "${_protoc_arguments}")
    list(APPEND _sources "${_autocmake_proto_source}")
    list(APPEND _headers "${_autocmake_proto_header}")
  endforeach()
  _autocmake_proto_attach("${target}" "${_sources}" "${_headers}" "${_arg_DEPENDENCIES}")
  _autocmake_proto_config("${_layout_root}")
  set(${PROJECT_NAME}_PROTO_PATH "${${PROJECT_NAME}_PROTO_PATH}" PARENT_SCOPE)
  set(${PROJECT_NAME}_PYTHON_PATH "${${PROJECT_NAME}_PYTHON_PATH}" PARENT_SCOPE)
endfunction()
