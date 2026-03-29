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

project(autolink LANGUAGES CXX)

# 默认使用 Release 构建，可以通过 -DCMAKE_BUILD_TYPE=Debug 覆盖
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# build options
option(AUTOLINK_BUILD_TOOLS "Build autolink tools" ON)
option(AUTOLINK_BUILD_TEST "Build unit tests" ON)
option(AUTOLINK_BUILD_EXAMPLES "Build examples" ON)
option(AUTOLINK_BUILD_PYTHON "Build Python bindings" ON)
option(AUTOLINK_BUILD_DOCS "Build documentation" ON)
option(AUTOLINK_FORCE_TCMALLOC_WITH_PYTHON
  "Force linking tcmalloc into autolink even when Python bindings are enabled" OFF)

# output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# root directory
set(AUTOLINK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${AUTOLINK_ROOT_DIR})

# include modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_autonomy_project()

if(AUTOLINK_BUILD_TEST)
  enable_testing()
  find_package(GMock REQUIRED)
endif()

# find dependencies
find_package(Protobuf REQUIRED)
find_package(abseil CONFIG QUIET)
find_package(FastDDS QUIET)
find_package(Glog REQUIRED)
find_package(Gperftools REQUIRED)
find_package(TinyXML2 QUIET)
find_library(GFLAGS_LIB NAMES gflags)
find_library(UUID_LIB NAMES uuid)

# libfastrtps depends on libfastcdr.so.N; some loaders resolve that using only a
# narrow RPATH. Embed the real directories of the FastDDS/FastCDR libs used at
# link time (skips non-path imported target names like "fastcdr").
set(_AUTOLINK_FASTDDS_RPATH_DIRS "")
foreach(_libvar IN ITEMS FastDDS_fastcdr_LIBRARY FastDDS_fastdds_LIBRARY)
  if(DEFINED ${_libvar})
    set(_libpath "${${_libvar}}")
    if(EXISTS "${_libpath}")
      get_filename_component(_real "${_libpath}" REALPATH)
      get_filename_component(_libdir "${_real}" DIRECTORY)
      list(APPEND _AUTOLINK_FASTDDS_RPATH_DIRS "${_libdir}")
    endif()
  endif()
endforeach()
if(_AUTOLINK_FASTDDS_RPATH_DIRS)
  list(REMOVE_DUPLICATES _AUTOLINK_FASTDDS_RPATH_DIRS)
  list(APPEND CMAKE_BUILD_RPATH ${_AUTOLINK_FASTDDS_RPATH_DIRS})
  list(APPEND CMAKE_INSTALL_RPATH ${_AUTOLINK_FASTDDS_RPATH_DIRS})
endif()

# add subdirectories（proto 已合并进 autolink/CMakeLists.txt）
add_subdirectory(autolink)

# add examples  
if(AUTOLINK_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# add docs
if(AUTOLINK_BUILD_DOCS)
  add_subdirectory(docs)
endif()

# -----------------------------------------------------------------------------
# install: sudo make install 或 cmake --build build --target install
# -----------------------------------------------------------------------------
include(GNUInstallDirs)

# 库与可执行文件（autolink_proto 已静态链接进 autolink，仅安装 libautolink.so）
install(TARGETS autolink
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS autolink_mainboard
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 头文件（源码中的 .hpp/.h）：autolink/ 即 add_subdirectory(autolink) 所在目录，含 base/、tools/ 等
install(DIRECTORY autolink/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/autolink
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 生成的 proto 头文件（.pb.h）
install(DIRECTORY ${CMAKE_BINARY_DIR}/autolink/proto/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/autolink/proto
  FILES_MATCHING PATTERN "*.pb.h"
)

# config files -> /usr/local/share/autolink/conf/*.conf
install(DIRECTORY autolink/conf/
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/autolink/conf
  FILES_MATCHING PATTERN "*.conf"
)
