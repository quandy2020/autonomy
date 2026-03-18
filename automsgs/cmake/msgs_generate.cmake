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

# =============================================================================
# This file provides CMake helpers for generating C++ / Python code from
# .proto definitions in the automsgs project.
#
# 它参考了 gz-msgs 的 gz_msgs_generate.cmake，但去掉了对 gz-cmake 的依赖，
# 仅保留通用的 protoc + Python 生成逻辑，并适配 automsgs 的命名和目录结构。
# =============================================================================

include(CMakeParseArguments)

# -----------------------------------------------------------------------------
# Utility: convert proto package into filesystem path
# (e.g. automsgs.msgs -> automsgs/msgs)
# -----------------------------------------------------------------------------
function(_automsgs_proto_pkg_to_path PROTO_PACKAGE PROTO_PACKAGE_PATH)
  if (PROTO_PACKAGE)
    string(REPLACE "." "/" PACKAGE_PATH ${PROTO_PACKAGE})
  else()
    set(PACKAGE_PATH ".")
  endif()
  set(${PROTO_PACKAGE_PATH} ${PACKAGE_PATH} PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------
# Utility: convert proto package into underscore string
# (e.g. automsgs.msgs -> automsgs_msgs)
# -----------------------------------------------------------------------------
function(_automsgs_proto_pkg_to_string PROTO_PACKAGE PROTO_PACKAGE_STRING)
  if (PROTO_PACKAGE)
    string(REPLACE "." "_" PACKAGE_STRING ${PROTO_PACKAGE})
  else()
    set(PACKAGE_STRING "")
  endif()
  set(${PROTO_PACKAGE_STRING} ${PACKAGE_STRING} PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------
# automsgs_msgs_generate_messages_impl
#
# 低层接口：针对一组 .proto 文件生成 C++ / Python / 工厂代码。
# 典型调用方式应由更高层 CMakeLists 封装。
#
# 关键参数（基本沿用 gz-msgs）：
#   PYTHON_INTERPRETER  - Python 解释器 target 或路径
#   PROTOC_EXEC         - protoc target 或可执行
#   PROTO_PACKAGE       - proto 包名（如 automsgs.msgs）
#   MSGS_GEN_SCRIPT     - 消息生成 Python 脚本路径
#   GZ_PROTOC_PLUGIN    - 自定义 protoc 插件可执行（此处仍沿用变量名，方便以后共用脚本）
#   FACTORY_GEN_SCRIPT  - 工厂代码生成脚本路径
#   PROTO_PATH          - proto 根目录
#   DEPENDENCY_DESCRIPTIONS - 依赖的描述符集（可选）
#   DLLEXPORT_DECL      - 生成 C++ 时使用的可见性宏（可选）
#   OUTPUT_DIRECTORY    - 生成代码输出目录
#   OUTPUT_SOURCES      -（输出变量名）C++ 源文件列表
#   OUTPUT_HEADERS      -（输出变量名）C++ 头文件列表
#   OUTPUT_DETAIL_HEADERS -（输出变量名）detail 头文件列表
#   OUTPUT_PYTHON       -（输出变量名）Python 代码列表
#   INPUT_PROTOS        - 输入 .proto 文件列表
#   PROTO_SOURCE_DIR   - 可选，proto 源目录；若设置则用 PROTO_PATH 下的 copy 布局，并传 INPUT_PATH_REL
#   PROTO_STAMP        - 可选，copy 生成的 stamp 文件，加入各 protoc 的 DEPENDS
# -----------------------------------------------------------------------------
function(automsgs_msgs_generate_messages_impl)
  set(options "")
  set(oneValueArgs
    PYTHON_INTERPRETER
    PROTOC_EXEC
    PROTO_PACKAGE
    MSGS_GEN_SCRIPT
    GZ_PROTOC_PLUGIN
    FACTORY_GEN_SCRIPT
    PROTO_PATH
    PROTO_SOURCE_DIR
    PROTO_STAMP
    DEPENDENCY_DESCRIPTIONS
    DLLEXPORT_DECL
    OUTPUT_DIRECTORY
    OUTPUT_SOURCES
    OUTPUT_HEADERS
    OUTPUT_DETAIL_HEADERS
    OUTPUT_PYTHON
  )
  set(multiValueArgs INPUT_PROTOS)

  cmake_parse_arguments(AM_GEN "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # 默认值
  if (NOT DEFINED AM_GEN_PYTHON_INTERPRETER)
    set(AM_GEN_PYTHON_INTERPRETER Python3::Interpreter)
  endif()
  if(AM_GEN_PYTHON_INTERPRETER STREQUAL "Python3::Interpreter" AND NOT TARGET Python3::Interpreter)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
  endif()

  if (NOT DEFINED AM_GEN_PROTOC_EXEC)
    set(AM_GEN_PROTOC_EXEC protobuf::protoc)
  endif()

  _automsgs_proto_pkg_to_string(${AM_GEN_PROTO_PACKAGE} gen_dir)
  _automsgs_proto_pkg_to_path(${AM_GEN_PROTO_PACKAGE} proto_package_dir)

  set(output_directory ${AM_GEN_OUTPUT_DIRECTORY})
  file(MAKE_DIRECTORY ${output_directory})
  file(MAKE_DIRECTORY ${output_directory}/${proto_package_dir})

  # 这里假定你将来会提供 automsgs_protoc() 函数（类似 gz_msgs_protoc），
  # 负责实际调用 protoc + Python 生成 C++/Python 代码和 .pb_index 文件。

  foreach(proto_file ${AM_GEN_INPUT_PROTOS})
    if(NOT COMMAND automsgs_protoc)
      message(FATAL_ERROR
        "automsgs_protoc() is not defined. Please provide a msgs_protoc.cmake with this function.")
    endif()
    set(input_path_rel "")
    set(proto_stamp "")
    if(DEFINED AM_GEN_PROTO_SOURCE_DIR AND AM_GEN_PROTO_SOURCE_DIR)
      file(RELATIVE_PATH rel "${AM_GEN_PROTO_SOURCE_DIR}" "${proto_file}")
      set(input_path_rel "automsgs/${rel}")
    endif()
    if(DEFINED AM_GEN_PROTO_STAMP AND AM_GEN_PROTO_STAMP)
      set(proto_stamp "${AM_GEN_PROTO_STAMP}")
    endif()
    automsgs_protoc(
      PYTHON_INTERPRETER  ${AM_GEN_PYTHON_INTERPRETER}
      MSGS_GEN_SCRIPT     ${AM_GEN_MSGS_GEN_SCRIPT}
      PROTO_PACKAGE       ${AM_GEN_PROTO_PACKAGE}
      INPUT_PROTO         ${proto_file}
      INPUT_PATH_REL      ${input_path_rel}
      PROTO_STAMP         ${proto_stamp}
      PROTOC_EXEC         ${AM_GEN_PROTOC_EXEC}
      GZ_PROTOC_PLUGIN    ${AM_GEN_GZ_PROTOC_PLUGIN}
      PROTO_PATH          ${AM_GEN_PROTO_PATH}
      DEPENDENCY_PROTO_DESCS ${AM_GEN_DEPENDENCY_DESCRIPTIONS}

      GENERATE_CPP
      DLLEXPORT_DECL      ${AM_GEN_DLLEXPORT_DECL}
      OUTPUT_CPP_HH_VAR   ${AM_GEN_OUTPUT_HEADERS}
      OUTPUT_DETAIL_CPP_HH_VAR ${AM_GEN_OUTPUT_DETAIL_HEADERS}
      OUTPUT_CPP_CC_VAR   ${AM_GEN_OUTPUT_SOURCES}
      OUTPUT_CPP_DIR      ${output_directory}

      GENERATE_PYTHON
      OUTPUT_PYTHON_VAR   ${AM_GEN_OUTPUT_PYTHON}
      OUTPUT_PYTHON_DIR   ${output_directory}/python/
    )
  endforeach()

  # 如果调用方同时提供了 FACTORY_GEN_SCRIPT，则自动生成工厂注册代码
  if (AM_GEN_FACTORY_GEN_SCRIPT)
    if(NOT COMMAND automsgs_msgs_factory)
      message(FATAL_ERROR
        "automsgs_msgs_factory() is not defined. Please include msgs_factory.cmake before calling automsgs_msgs_generate_messages_impl().")
    endif()

    automsgs_msgs_factory(
      PYTHON_INTERPRETER  ${AM_GEN_PYTHON_INTERPRETER}
      FACTORY_GEN_SCRIPT  ${AM_GEN_FACTORY_GEN_SCRIPT}
      PROTO_PACKAGE       ${AM_GEN_PROTO_PACKAGE}
      INPUT_PROTOS        ${AM_GEN_INPUT_PROTOS}
      PROTO_PATH          ${AM_GEN_PROTO_PATH}
      OUTPUT_CPP_DIR      ${output_directory}
      OUTPUT_CPP_HH_VAR   ${AM_GEN_OUTPUT_HEADERS}
      OUTPUT_CPP_CC_VAR   ${AM_GEN_OUTPUT_SOURCES}
    )
  endif()

  # 标记生成文件为 GENERATED
  set_source_files_properties(
    ${${AM_GEN_OUTPUT_SOURCES}}
    ${${AM_GEN_OUTPUT_HEADERS}}
    ${${AM_GEN_OUTPUT_DETAIL_HEADERS}}
    PROPERTIES GENERATED TRUE)

  if(NOT MSVC)
    set_source_files_properties(${${AM_GEN_OUTPUT_SOURCES}}
      PROPERTIES COMPILE_FLAGS "-Wno-switch-default -Wno-float-equal")
  endif()

  # 将结果传递回调用者
  set(${AM_GEN_OUTPUT_SOURCES}        ${${AM_GEN_OUTPUT_SOURCES}}        PARENT_SCOPE)
  set(${AM_GEN_OUTPUT_HEADERS}        ${${AM_GEN_OUTPUT_HEADERS}}        PARENT_SCOPE)
  set(${AM_GEN_OUTPUT_DETAIL_HEADERS} ${${AM_GEN_OUTPUT_DETAIL_HEADERS}} PARENT_SCOPE)
  set(${AM_GEN_OUTPUT_PYTHON}         ${${AM_GEN_OUTPUT_PYTHON}}         PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------
# automsgs_msgs_generate_desc_impl
#
# 生成一个描述符集文件（.desc），类似 gz-msgs 中的 gz_msgs_generate_desc_impl。
# 目前实现为直接调用 protoc --descriptor_set_out。
# -----------------------------------------------------------------------------
function(automsgs_msgs_generate_desc_impl)
  set(options "")
  set(oneValueArgs
    PROTOC_EXEC
    PROTO_PATH
    OUTPUT_DIRECTORY
    OUTPUT_FILENAME
  )
  set(multiValueArgs INPUT_PROTOS DEPENDENCY_DESCRIPTIONS)

  cmake_parse_arguments(AM_DESC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if (NOT DEFINED AM_DESC_PROTOC_EXEC)
    set(AM_DESC_PROTOC_EXEC protobuf::protoc)
  endif()

  set(output_file ${AM_DESC_OUTPUT_DIRECTORY}/${AM_DESC_OUTPUT_FILENAME})
  file(MAKE_DIRECTORY ${AM_DESC_OUTPUT_DIRECTORY})

  set(ARGS)
  list(APPEND ARGS -I${AM_DESC_PROTO_PATH})
  list(APPEND ARGS --descriptor_set_out=${output_file})
  list(APPEND ARGS --include_imports)

  foreach(dep ${AM_DESC_DEPENDENCY_DESCRIPTIONS})
    list(APPEND ARGS --descriptor_set_in="${dep}")
  endforeach()

  list(APPEND ARGS ${AM_DESC_INPUT_PROTOS})

  add_custom_command(
    OUTPUT ${output_file}
    COMMAND ${AM_DESC_PROTOC_EXEC}
    ARGS ${ARGS}
    DEPENDS ${AM_DESC_INPUT_PROTOS}
    COMMENT "Generating descriptor set ${output_file}"
  )
endfunction()

