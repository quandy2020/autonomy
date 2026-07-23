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

##################################################
# Utility: convert proto package to filesystem path
# (e.g. automsgs.msgs -> automsgs/msgs)
function(_automsgs_proto_pkg_to_path PROTO_PACKAGE PROTO_PACKAGE_PATH)
  if (PROTO_PACKAGE)
    string(REPLACE "." "/" PACKAGE_PATH ${PROTO_PACKAGE})
  else()
    set(PACKAGE_PATH ".")
  endif()
  set(${PROTO_PACKAGE_PATH} ${PACKAGE_PATH} PARENT_SCOPE)
endfunction()

##################################################
# Utility: convert proto package + file to a unique string
# (e.g. automsgs.msgs.foobar -> automsgs_msgs_foobar)
function(_automsgs_proto_to_unique PROTO_FILE PROTO_PACKAGE UNIQUE_NAME)
  get_filename_component(FIL_WE ${PROTO_FILE} NAME_WE)
  if (PROTO_PACKAGE)
    string(REPLACE "." "_" PACKAGE_STRING ${PROTO_PACKAGE})
    set(${UNIQUE_NAME} "${PACKAGE_STRING}_${FIL_WE}" PARENT_SCOPE)
  else()
    set(${UNIQUE_NAME} "${FIL_WE}" PARENT_SCOPE)
  endif()
endfunction()

##################################################
# automsgs_msgs_factory
#
# 生成针对一组 .proto 文件的工厂注册代码，模式参考 gz-msgs 的
# gz_msgs_factory.cmake，但去掉了对 gz-cmake 的依赖，仅负责：
#  - 调用 Python 工具脚本
#  - 生成 MessageTypes.hh 和 register.cc
#  - 把生成文件路径追加到调用方指定的变量中
#
# 用法：
#
#  automsgs_msgs_factory(
#    FACTORY_GEN_SCRIPT  <python 脚本路径>
#    PROTO_PACKAGE       <proto 包名，如 "automsgs.msgs">
#    PYTHON_INTERPRETER  <Python3::Interpreter 或路径，可选>
#    OUTPUT_CPP_DIR      <生成 C++ 文件输出目录>
#    OUTPUT_CPP_HH_VAR   <CMake 变量名，用于收集头文件路径>
#    OUTPUT_CPP_CC_VAR   <CMake 变量名，用于收集源文件路径>
#    INPUT_PROTOS        <proto 列表>
#    PROTO_PATH          <proto 根目录（传给生成脚本）>
#  )
function(automsgs_msgs_factory)
  set(options "")
  set(oneValueArgs
    FACTORY_GEN_SCRIPT
    PROTO_PACKAGE
    PYTHON_INTERPRETER
    OUTPUT_CPP_DIR
    OUTPUT_CPP_HH_VAR
    OUTPUT_CPP_CC_VAR)
  set(multiValueArgs INPUT_PROTOS PROTO_PATH)

  cmake_parse_arguments(AM_FACTORY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # 默认 Python 解释器
  if (NOT DEFINED AM_FACTORY_PYTHON_INTERPRETER)
    set(AM_FACTORY_PYTHON_INTERPRETER Python3::Interpreter)
  endif()

  if(AM_FACTORY_PYTHON_INTERPRETER STREQUAL "Python3::Interpreter" AND NOT TARGET Python3::Interpreter)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
  endif()

  _automsgs_proto_pkg_to_path(${AM_FACTORY_PROTO_PACKAGE} proto_package_dir)

  set(output_header "${AM_FACTORY_OUTPUT_CPP_DIR}/${proto_package_dir}/MessageTypes.hh")
  set(output_source "${AM_FACTORY_OUTPUT_CPP_DIR}/${proto_package_dir}/register.cc")

  # 将生成的文件加入调用者收集变量
  list(APPEND ${AM_FACTORY_OUTPUT_CPP_HH_VAR} ${output_header})
  list(APPEND ${AM_FACTORY_OUTPUT_CPP_CC_VAR} ${output_source})

  list(APPEND output_files ${output_header})
  list(APPEND output_files ${output_source})

  set(${AM_FACTORY_OUTPUT_CPP_HH_VAR} ${${AM_FACTORY_OUTPUT_CPP_HH_VAR}} PARENT_SCOPE)
  set(${AM_FACTORY_OUTPUT_CPP_CC_VAR} ${${AM_FACTORY_OUTPUT_CPP_CC_VAR}} PARENT_SCOPE)

  # 依赖于各 proto 对应的 index 文件（如果你的生成脚本需要，可根据需要调整）
  set(depends_index)
  foreach(proto_file ${AM_FACTORY_INPUT_PROTOS})
    _automsgs_proto_to_unique(${proto_file} ${AM_FACTORY_PROTO_PACKAGE} UNIQUE_NAME)
    set(input_index "${AM_FACTORY_OUTPUT_CPP_DIR}/${UNIQUE_NAME}.pb_index")
    list(APPEND depends_index ${input_index})
  endforeach()

  set(GENERATE_ARGS
    --output-cpp-path "${AM_FACTORY_OUTPUT_CPP_DIR}"
    --proto-package "${AM_FACTORY_PROTO_PACKAGE}"
    --proto-path "${AM_FACTORY_PROTO_PATH}"
    --protos "${AM_FACTORY_INPUT_PROTOS}"
  )

  add_custom_command(
    OUTPUT ${output_files}
    COMMAND ${AM_FACTORY_PYTHON_INTERPRETER}
    ARGS ${AM_FACTORY_FACTORY_GEN_SCRIPT} ${GENERATE_ARGS}
    DEPENDS ${depends_index}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Running automsgs factory generator"
    VERBATIM
  )

endfunction()

