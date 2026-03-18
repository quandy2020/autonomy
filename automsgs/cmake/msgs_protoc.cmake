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
# msgs_protoc.cmake
#
# 提供 automsgs_protoc()，用于对单个 .proto 文件调用 protoc + Python 生成脚本，
# 生成 C++/Python 代码以及可选的索引文件。
# 该实现参考 gz-msgs 的 gz_msgs_protoc.cmake，去掉了对 gz-cmake 的依赖，
# 并适配 automsgs 前缀和目录结构。
# =============================================================================

include(CMakeParseArguments)

# 将 proto 包名转换为路径：automsgs.msgs -> automsgs/msgs
function(_automsgs_proto_pkg_to_path PROTO_PACKAGE PROTO_PACKAGE_PATH)
  if (PROTO_PACKAGE)
    string(REPLACE "." "/" PACKAGE_PATH ${PROTO_PACKAGE})
  else()
    set(PACKAGE_PATH ".")
  endif()
  set(${PROTO_PACKAGE_PATH} ${PACKAGE_PATH} PARENT_SCOPE)
endfunction()

# 将 proto 包名与文件组合成唯一字符串：automsgs.msgs.foo -> automsgs_msgs_foo
function(_automsgs_proto_to_unique PROTO_FILE PROTO_PACKAGE UNIQUE_NAME)
  get_filename_component(FIL_WE ${PROTO_FILE} NAME_WE)
  if (PROTO_PACKAGE)
    string(REPLACE "." "_" PACKAGE_STRING ${PROTO_PACKAGE})
    set(${UNIQUE_NAME} "${PACKAGE_STRING}_${FIL_WE}" PARENT_SCOPE)
  else()
    set(${UNIQUE_NAME} "${FIL_WE}" PARENT_SCOPE)
  endif()
endfunction()

# -----------------------------------------------------------------------------
# automsgs_protoc
#
# 选项：
#   GENERATE_CPP      - 生成 C++ 代码
#   GENERATE_PYTHON   - 生成 Python 代码
# 单值参数：
#   PYTHON_INTERPRETER  - Python 解释器 target 或路径
#   MSGS_GEN_SCRIPT     - Python 消息生成脚本
#   PROTO_PACKAGE       - proto 包名（如 automsgs.msgs）
#   PROTOC_EXEC         - protoc target 或可执行
#   GZ_PROTOC_PLUGIN    - 自定义 protoc 插件可执行（名称沿用，便于共用脚本）
#   DLLEXPORT_DECL      - C++ 代码中的导出宏（可选）
#   INPUT_PROTO         - 输入 .proto 文件（源路径，用于 DEPENDS）
#   INPUT_PATH_REL      - 可选，传给脚本的 --input-path（相对 PROTO_PATH，用于 copy 布局）
#   OUTPUT_CPP_DIR      - C++ 输出目录
#   OUTPUT_CPP_HH_VAR   - CMake 变量名：收集生成的头文件
#   OUTPUT_DETAIL_CPP_HH_VAR - CMake 变量名：收集生成的 detail 头文件
#   OUTPUT_CPP_CC_VAR   - CMake 变量名：收集生成的源文件
#   OUTPUT_PYTHON_DIR   - Python 输出目录
#   OUTPUT_PYTHON_VAR   - CMake 变量名：收集生成的 Python 文件
#   PROTO_STAMP         - 可选，copy 生成的 stamp 文件，加入 DEPENDS
# 多值参数：
#   PROTO_PATH          - 传给生成脚本 / protoc 的 proto 搜索路径
#   DEPENDENCY_PROTO_DESCS - 依赖的描述符集（可选）
# -----------------------------------------------------------------------------
function(automsgs_protoc)
  set(options GENERATE_CPP GENERATE_PYTHON)
  set(oneValueArgs
    PYTHON_INTERPRETER
    MSGS_GEN_SCRIPT
    PROTO_PACKAGE
    PROTOC_EXEC
    GZ_PROTOC_PLUGIN
    DLLEXPORT_DECL
    INPUT_PROTO
    INPUT_PATH_REL
    OUTPUT_CPP_DIR
    OUTPUT_CPP_HH_VAR
    OUTPUT_DETAIL_CPP_HH_VAR
    OUTPUT_CPP_CC_VAR
    OUTPUT_PYTHON_DIR
    OUTPUT_PYTHON_VAR
    PROTO_STAMP)
  set(multiValueArgs PROTO_PATH DEPENDENCY_PROTO_DESCS)

  cmake_parse_arguments(AM_PROTOC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # 默认 Python、protoc
  if (NOT DEFINED AM_PROTOC_PYTHON_INTERPRETER)
    set(AM_PROTOC_PYTHON_INTERPRETER Python3::Interpreter)
  endif()
  if(AM_PROTOC_PYTHON_INTERPRETER STREQUAL "Python3::Interpreter" AND NOT TARGET Python3::Interpreter)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
  endif()

  if (NOT DEFINED AM_PROTOC_PROTOC_EXEC)
    set(AM_PROTOC_PROTOC_EXEC protobuf::protoc)
  endif()

  get_filename_component(ABS_FIL ${AM_PROTOC_INPUT_PROTO} ABSOLUTE)
  get_filename_component(FIL_WE ${AM_PROTOC_INPUT_PROTO} NAME_WE)

  _automsgs_proto_pkg_to_path(${AM_PROTOC_PROTO_PACKAGE} proto_package_dir)

  set(output_files)

  # C++ 生成路径
  if(AM_PROTOC_GENERATE_CPP)
    set(output_header        "${AM_PROTOC_OUTPUT_CPP_DIR}/${proto_package_dir}/${FIL_WE}.pb.h")
    set(output_detail_header "${AM_PROTOC_OUTPUT_CPP_DIR}/${proto_package_dir}/details/${FIL_WE}.pb.h")
    set(output_source        "${AM_PROTOC_OUTPUT_CPP_DIR}/${proto_package_dir}/${FIL_WE}.pb.cc")

    _automsgs_proto_to_unique(${AM_PROTOC_INPUT_PROTO} ${AM_PROTOC_PROTO_PACKAGE} UNIQUE_NAME)
    set(output_index "${AM_PROTOC_OUTPUT_CPP_DIR}/${UNIQUE_NAME}.pb_index")

    list(APPEND ${AM_PROTOC_OUTPUT_CPP_HH_VAR}        ${output_header})
    list(APPEND ${AM_PROTOC_OUTPUT_DETAIL_CPP_HH_VAR} ${output_detail_header})
    list(APPEND ${AM_PROTOC_OUTPUT_CPP_CC_VAR}        ${output_source})

    list(APPEND output_files
      ${output_header}
      ${output_detail_header}
      ${output_source}
      ${output_index})

    set(${AM_PROTOC_OUTPUT_CPP_HH_VAR}        ${${AM_PROTOC_OUTPUT_CPP_HH_VAR}}        PARENT_SCOPE)
    set(${AM_PROTOC_OUTPUT_DETAIL_CPP_HH_VAR} ${${AM_PROTOC_OUTPUT_DETAIL_CPP_HH_VAR}} PARENT_SCOPE)
    set(${AM_PROTOC_OUTPUT_CPP_CC_VAR}        ${${AM_PROTOC_OUTPUT_CPP_CC_VAR}}        PARENT_SCOPE)
  endif()

  # Python 生成路径
  if(AM_PROTOC_GENERATE_PYTHON)
    file(MAKE_DIRECTORY ${AM_PROTOC_OUTPUT_PYTHON_DIR})
    # 注意：protobuf Python 生成文件统一使用 _pb2.py 后缀
    set(output_python "${AM_PROTOC_OUTPUT_PYTHON_DIR}${proto_package_dir}/${FIL_WE}_pb2.py")
    list(APPEND ${AM_PROTOC_OUTPUT_PYTHON_VAR} ${output_python})
    list(APPEND output_files ${output_python})
    set(${AM_PROTOC_OUTPUT_PYTHON_VAR} ${${AM_PROTOC_OUTPUT_PYTHON_VAR}} PARENT_SCOPE)
  endif()

  # 传给脚本的 input-path：若指定 INPUT_PATH_REL 则用（相对 PROTO_PATH），否则用源文件绝对路径
  if(DEFINED AM_PROTOC_INPUT_PATH_REL AND AM_PROTOC_INPUT_PATH_REL)
    set(SCRIPT_INPUT_PATH "${AM_PROTOC_INPUT_PATH_REL}")
  else()
    set(SCRIPT_INPUT_PATH "${ABS_FIL}")
  endif()
  # 解析 PROTOC_EXEC：若为 target 则用生成器表达式取可执行路径
  if(TARGET "${AM_PROTOC_PROTOC_EXEC}")
    set(AM_PROTOC_PROTOC_EXEC_FILE "$<TARGET_FILE:${AM_PROTOC_PROTOC_EXEC}>")
  else()
    set(AM_PROTOC_PROTOC_EXEC_FILE "${AM_PROTOC_PROTOC_EXEC}")
  endif()
  set(GENERATE_ARGS
    --protoc-exec       "${AM_PROTOC_PROTOC_EXEC_FILE}"
    --generator-bin     "${AM_PROTOC_GZ_PROTOC_PLUGIN}"
    --proto-path        "${AM_PROTOC_PROTO_PATH}"
    --input-path        "${SCRIPT_INPUT_PATH}"
  )

  if(AM_PROTOC_DEPENDENCY_PROTO_DESCS)
    list(APPEND GENERATE_ARGS
      --dependency-proto-descs "${AM_PROTOC_DEPENDENCY_PROTO_DESCS}")
  endif()

  if(AM_PROTOC_DLLEXPORT_DECL)
    list(APPEND GENERATE_ARGS
      --dllexport-decl "${AM_PROTOC_DLLEXPORT_DECL}")
  endif()

  if(AM_PROTOC_GENERATE_CPP)
    list(APPEND GENERATE_ARGS
      --generate-cpp
      --output-cpp-path "${AM_PROTOC_OUTPUT_CPP_DIR}")
  endif()

  if(AM_PROTOC_GENERATE_PYTHON)
    list(APPEND GENERATE_ARGS
      --generate-python
      --output-python-path "${AM_PROTOC_OUTPUT_PYTHON_DIR}")
  endif()

  set(protoc_depends ${ABS_FIL})
  if(DEFINED AM_PROTOC_PROTO_STAMP AND AM_PROTOC_PROTO_STAMP)
    list(APPEND protoc_depends "${AM_PROTOC_PROTO_STAMP}")
  endif()
  add_custom_command(
    OUTPUT ${output_files}
    COMMAND ${AM_PROTOC_PYTHON_INTERPRETER}
    ARGS ${AM_PROTOC_MSGS_GEN_SCRIPT} ${GENERATE_ARGS}
    DEPENDS ${protoc_depends}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Running automsgs_protoc on ${AM_PROTOC_INPUT_PROTO}"
    VERBATIM
  )

endfunction()

