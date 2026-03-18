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
add_subdirectory(tools)
add_subdirectory(conf)
add_subdirectory(python)
add_subdirectory(core)
add_subdirectory(examples)
add_subdirectory(test)
