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

cmake_minimum_required(VERSION 3.14)

project(autoviz LANGUAGES CXX)

# build options
option(AUTOVIZ_BUILD_TEST "Build unit tests" ON)

# output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# find dependencies
find_package(autolink REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)

# collect files
file(GLOB_RECURSE ALL_LIBRARY_HDRS "autoviz/*.hpp")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "autoviz/*.cpp")
file(GLOB_RECURSE ALL_TESTS "autoviz/*_test.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})

# add executable
add_executable(autoviz ${ALL_LIBRARY_SRCS})
target_link_libraries(autoviz autolink gflags glog)
target_include_directories(autoviz PUBLIC ${ALL_LIBRARY_HDRS})

# test
if (AUTOVIZ_BUILD_TEST)
   # TODO: add test
endif()

# install
install(TARGETS autoviz DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)