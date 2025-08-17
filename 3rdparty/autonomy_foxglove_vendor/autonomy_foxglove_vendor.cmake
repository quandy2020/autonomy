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

cmake_minimum_required(VERSION 3.5)

project(autonomy_foxglove_sdk_vendor)

macro(build_foxglove)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -std=c++17 -w")
  list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS} -w")
  list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  set(should_log OFF)

  include(ExternalProject)
  ExternalProject_Add(foxglove
    URL https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv0.10.0/foxglove-v0.10.0-cpp-aarch64-unknown-linux-gnu.zip
    URL_MD5 81e2379e3a08160c023779eab0fe5c5764ace3b30c8727b0030ac7dc8b415016
    TIMEOUT 600
    LOG_CONFIGURE ${should_log}
    LOG_BUILD ${should_log}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/foxglove_install
      ${extra_cmake_args}
      -Wno-dev
  )

  # 创建可供外部使用的导入目标
  ExternalProject_Get_Property(foxglove_sdk install_dir)
  add_library(Foxglove::SDK STATIC IMPORTED)
  set_target_properties(Foxglove::SDK PROPERTIES
      IMPORTED_LOCATION ${install_dir}/lib/libfoxglove.a  # 根据实际库名调整
      INTERFACE_INCLUDE_DIRECTORIES ${install_dir}/include
  )
  add_dependencies(Foxglove::SDK foxglove_sdk)

endmacro()

build_foxglove()

# # 提供包配置文件（可选）
# export(TARGETS Foxglove::SDK FILE FoxgloveSDKConfig.cmake)
