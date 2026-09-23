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

# Message package. proto/ generates code and libautomsgs.
# The generator is the Python protoc wrapper, not autocmake_protobuf:
# it also writes detail headers and the import layout "automsgs/...".

cmake_minimum_required(VERSION 3.20)

project(automsgs VERSION 1.0.0 LANGUAGES CXX)

find_package(autocmake CONFIG QUIET)
if(NOT autocmake_FOUND)
  set(AUTOCMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autocmake")
  include("${AUTOCMAKE_DIR}/Autocmake.cmake")
endif()

option(AUTOMSGS_BUILD_TOOLS "Build automsgs command-line tools" ON)
option(AUTOMSGS_BUILD_PYTHON "Build automsgs Python bindings" ON)
option(AUTOMSGS_BUILD_EXAMPLES "Build automsgs examples" OFF)
option(AUTOMSGS_BUILD_TESTS "Build automsgs tests" OFF)

if(DEFINED AUTOMSGS_BUILD_TESTS AND NOT AUTOMSGS_BUILD_TESTS)
  set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
endif()

# Pin Protobuf 3.19 before package.xml resolves it.
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules")
include(EnsureProtobuf319)

autocmake_project()
autocmake_build(QUIT)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT DEFINED AUTOMSGS_PYTHON_INSTALL_PATH)
  set(AUTOMSGS_PYTHON_INSTALL_PATH "${CMAKE_INSTALL_LIBDIR}/python"
    CACHE PATH "Install path for Python modules")
endif()

set(AUTOMSGS_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(proto)
get_directory_property(_automsgs_parent PARENT_DIRECTORY)
if(_automsgs_parent)
  foreach(_var IN ITEMS AUTOMSGS_PROTO_INCLUDE_DIR AUTOMSGS_PROTO_GEN_DIR)
    if(DEFINED ${_var})
      set(${_var} "${${_var}}" PARENT_SCOPE)
    endif()
  endforeach()
endif()

add_subdirectory(core)

# Exported autonomy modules link this library. It can sit in AutonomyExport
# as well as the autocmake export.
if(CMAKE_PROJECT_NAME STREQUAL "autonomy")
  install(TARGETS automsgs
    EXPORT AutonomyExport
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
add_subdirectory(conf)
if(AUTOMSGS_BUILD_TOOLS)
  add_subdirectory(tools)
endif()
if(AUTOMSGS_BUILD_PYTHON)
  add_subdirectory(python)
endif()
if(AUTOMSGS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
if(AUTOMSGS_BUILD_TESTS)
  add_subdirectory(test)
endif()

autocmake_package()
