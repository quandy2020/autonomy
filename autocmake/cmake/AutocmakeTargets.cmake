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

# @brief Add one library.
#
# Exported as ${PROJECT_NAME}::<target>. When <target> is the project name,
# ${PROJECT_NAME}::core names the same library.
#
# @param target Target name.
# @param DIRECTORY Directory to glob for sources, relative to the caller.
# @param SOURCES Source files. Required when DIRECTORY and INTERFACE are not set.
# @param DEPENDENCIES Targets linked PUBLIC.
# @param STATIC SHARED INTERFACE Library type. Default follows BUILD_SHARED_LIBS.
function(autocmake_library target)
  _autocmake_parse(_arg "STATIC;SHARED;INTERFACE" "DIRECTORY" "SOURCES;DEPENDENCIES" ${ARGN})

  set(_sources ${_arg_SOURCES})
  if(_arg_DIRECTORY)
    file(GLOB_RECURSE _globbed CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/${_arg_DIRECTORY}/*.c"
      "${CMAKE_CURRENT_SOURCE_DIR}/${_arg_DIRECTORY}/*.cc"
      "${CMAKE_CURRENT_SOURCE_DIR}/${_arg_DIRECTORY}/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/${_arg_DIRECTORY}/*.cxx")
    list(APPEND _sources ${_globbed})
  endif()

  if(_arg_INTERFACE)
    add_library(${target} INTERFACE)
    set(_scope INTERFACE)
  else()
    if(NOT _sources)
      message(FATAL_ERROR
        "autocmake_library(${target}): pass SOURCES or DIRECTORY")
    endif()
    if(_arg_STATIC)
      add_library(${target} STATIC ${_sources})
    elseif(_arg_SHARED)
      add_library(${target} SHARED ${_sources})
    else()
      add_library(${target} ${_sources})
    endif()
    set(_scope PUBLIC)
    set_target_properties(${target} PROPERTIES
      VERSION "${PROJECT_VERSION}"
      SOVERSION "${PROJECT_VERSION_MAJOR}")
  endif()

  if(CMAKE_CXX_STANDARD MATCHES "^(98|11|14|17|20|23)$")
    target_compile_features(${target} ${_scope} cxx_std_${CMAKE_CXX_STANDARD})
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_include_directories(${target} ${_scope}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  endif()

  if(_arg_DEPENDENCIES)
    target_link_libraries(${target} ${_scope} ${_arg_DEPENDENCIES})
  endif()

  _autocmake_install_target(${target})
  _autocmake_add_alias(${PROJECT_NAME}::${target} ${target})
  if(target STREQUAL PROJECT_NAME)
    _autocmake_add_alias(${PROJECT_NAME}::core ${target})
  endif()
endfunction()

# @brief Add one installed executable.
#
# @param name Target name.
# @param SOURCES Sources.
# @param DEPENDENCIES Targets linked PRIVATE.
function(autocmake_binary name)
  _autocmake_parse(_arg "" "" "SOURCES;DEPENDENCIES" ${ARGN})
  if(NOT _arg_SOURCES)
    message(FATAL_ERROR "autocmake_binary(${name}): SOURCES is required")
  endif()
  add_executable(${name} ${_arg_SOURCES})
  if(_arg_DEPENDENCIES)
    target_link_libraries(${name} PRIVATE ${_arg_DEPENDENCIES})
  endif()
  _autocmake_install_target(${name})
endfunction()
