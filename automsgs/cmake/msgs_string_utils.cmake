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
# msgs_string_utils.cmake
#
# 参考 gz-msgs 的 gz_msgs_string_utils.cmake，实现 automsgs 自己的
# proto 包名与路径/字符串工具函数，供 msgs_protoc.cmake、msgs_generate.cmake、
# msgs_factory.cmake 复用。
# =============================================================================

# -----------------------------------------------------------------------------
# _automsgs_proto_pkg_to_path
#   将 proto 包名转换为文件系统路径
#   例如：automsgs.msgs -> automsgs/msgs
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
# _automsgs_proto_pkg_to_string
#   将 proto 包名转换为下划线字符串
#   例如：automsgs.msgs -> automsgs_msgs
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
# _automsgs_proto_to_unique
#   将 proto 文件与包名组合成唯一字符串
#   例如：automsgs.msgs.foobar.proto -> automsgs_msgs_foobar
# -----------------------------------------------------------------------------
function(_automsgs_proto_to_unique PROTO_FILE PROTO_PACKAGE UNIQUE_NAME)
  # 获取不带扩展名和路径的文件名
  get_filename_component(FIL_WE ${PROTO_FILE} NAME_WE)
  if (PROTO_PACKAGE)
    string(REPLACE "." "_" PACKAGE_STRING ${PROTO_PACKAGE})
    set(${UNIQUE_NAME} "${PACKAGE_STRING}_${FIL_WE}" PARENT_SCOPE)
  else()
    set(${UNIQUE_NAME} "${FIL_WE}" PARENT_SCOPE)
  endif()
endfunction()

