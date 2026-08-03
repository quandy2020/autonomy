# Copyright 2025 The Openbot Authors （duyongquan）
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

cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(automsgs VERSION 1.0.0)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/modules")
include(EnsureProtobuf319)
autonomy_require_protobuf()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(NOT DEFINED AUTOMSGS_PYTHON_INSTALL_PATH)
  set(AUTOMSGS_PYTHON_INSTALL_PATH "${CMAKE_INSTALL_LIBDIR}/python" CACHE PATH "Install path for Python modules")
endif()

enable_testing()

#============================================================================
# Subdirectories
#============================================================================

# Install and manage .proto message definitions
add_subdirectory(proto)
# Re-export proto include/gen dirs so parent (autonomy) protoc can -I them.
if(DEFINED AUTOMSGS_PROTO_INCLUDE_DIR)
  set(AUTOMSGS_PROTO_INCLUDE_DIR "${AUTOMSGS_PROTO_INCLUDE_DIR}" PARENT_SCOPE)
endif()
if(DEFINED AUTOMSGS_PROTO_GEN_DIR)
  set(AUTOMSGS_PROTO_GEN_DIR "${AUTOMSGS_PROTO_GEN_DIR}" PARENT_SCOPE)
endif()
add_subdirectory(tools)
add_subdirectory(conf)
add_subdirectory(python)
add_subdirectory(core)
add_subdirectory(examples)
add_subdirectory(test)

#============================================================================
# CMake package export (find_package support for third-party projects)
#============================================================================
set(AUTOMSGS_INSTALL_CMAKEDIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/automsgs"
    CACHE PATH "Install path for automsgs CMake package files")

if(TARGET automsgs)
  install(EXPORT automsgsTargets
    FILE automsgsTargets.cmake
    NAMESPACE automsgs::
    DESTINATION "${AUTOMSGS_INSTALL_CMAKEDIR}")

  export(EXPORT automsgsTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/automsgsTargets.cmake"
    NAMESPACE automsgs::)
endif()

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/automsgs_config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/automsgsConfig.cmake"
  INSTALL_DESTINATION "${AUTOMSGS_INSTALL_CMAKEDIR}")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/automsgsConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/automsgsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/automsgsConfigVersion.cmake"
  DESTINATION "${AUTOMSGS_INSTALL_CMAKEDIR}")
