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

#[=======================================================================[.rst:
read_version_from_json
----------------------

从 version.json 文件读取版本信息并设置 CMake 变量

Synopsis
^^^^^^^^

.. code-block:: cmake

  read_version_from_json(<prefix> <version_file>)

Description
^^^^^^^^^^^

该函数从指定的 JSON 文件中读取版本信息，并设置以下 CMake 变量：

- ``<prefix>_MAJOR_VERSION`` - 主版本号
- ``<prefix>_MINOR_VERSION`` - 次版本号
- ``<prefix>_PATCH_VERSION`` - 修订版本号
- ``<prefix>_VERSION`` - 完整版本号 (major.minor.patch)
- ``<prefix>_SOVERSION`` - 共享库版本号 (major.minor)
- ``<prefix>_NAME`` - 项目名称（如果 JSON 中存在）

参数
^^^^

``<prefix>``
  变量前缀，例如 "AUTONOMY" 或 "AUTOLINK"

``<version_file>``
  version.json 文件的路径（相对或绝对路径）

示例
^^^^

.. code-block:: cmake

  # 读取项目根目录的 version.json
  read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")
  
  # 输出版本信息
  message(STATUS "${AUTONOMY_NAME} version: ${AUTONOMY_VERSION}")
  
  # 使用版本变量
  set_target_properties(mylib PROPERTIES 
    VERSION ${AUTONOMY_VERSION}
    SOVERSION ${AUTONOMY_SOVERSION}
  )

version.json 格式
^^^^^^^^^^^^^^^^^

.. code-block:: json

  {
    "version": "0.0.3",
    "major": 0,
    "minor": 0,
    "patch": 3,
    "name": "Autonomy",
    "description": "Autonomous Robot Development Framework",
    "build_date": "2025-11-20"
  }

#]=======================================================================]

function(read_version_from_json PREFIX VERSION_FILE)
  # 检查文件是否存在
  if(NOT EXISTS "${VERSION_FILE}")
    message(FATAL_ERROR "Version file not found: ${VERSION_FILE}")
  endif()

  # 读取 JSON 文件内容
  file(READ "${VERSION_FILE}" VERSION_JSON)
  
  # 验证 JSON 内容不为空
  if(NOT VERSION_JSON)
    message(FATAL_ERROR "Version file is empty: ${VERSION_FILE}")
  endif()

  # 解析主版本号 (支持换行和空格)
  string(REGEX MATCH "\"major\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${VERSION_JSON}")
  if(NOT DEFINED CMAKE_MATCH_1)
    message(FATAL_ERROR "Cannot find 'major' field in ${VERSION_FILE}")
  endif()
  set(_MAJOR ${CMAKE_MATCH_1})
  set(${PREFIX}_MAJOR_VERSION ${_MAJOR} PARENT_SCOPE)
  
  # 解析次版本号
  string(REGEX MATCH "\"minor\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${VERSION_JSON}")
  if(NOT DEFINED CMAKE_MATCH_1)
    message(FATAL_ERROR "Cannot find 'minor' field in ${VERSION_FILE}")
  endif()
  set(_MINOR ${CMAKE_MATCH_1})
  set(${PREFIX}_MINOR_VERSION ${_MINOR} PARENT_SCOPE)
  
  # 解析修订版本号
  string(REGEX MATCH "\"patch\"[ \t\r\n]*:[ \t\r\n]*([0-9]+)" _ "${VERSION_JSON}")
  if(NOT DEFINED CMAKE_MATCH_1)
    message(FATAL_ERROR "Cannot find 'patch' field in ${VERSION_FILE}")
  endif()
  set(_PATCH ${CMAKE_MATCH_1})
  set(${PREFIX}_PATCH_VERSION ${_PATCH} PARENT_SCOPE)
  
  # 解析项目名称（可选）
  string(REGEX MATCH "\"name\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _ "${VERSION_JSON}")
  if(CMAKE_MATCH_1)
    set(_NAME ${CMAKE_MATCH_1})
    set(${PREFIX}_NAME ${_NAME} PARENT_SCOPE)
  else()
    set(_NAME ${PREFIX})
    set(${PREFIX}_NAME ${PREFIX} PARENT_SCOPE)
  endif()
  
  # 解析描述（可选）
  string(REGEX MATCH "\"description\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _ "${VERSION_JSON}")
  if(CMAKE_MATCH_1)
    set(${PREFIX}_DESCRIPTION ${CMAKE_MATCH_1} PARENT_SCOPE)
  endif()
  
  # 解析构建日期（可选）
  string(REGEX MATCH "\"build_date\"[ \t\r\n]*:[ \t\r\n]*\"([^\"]+)\"" _ "${VERSION_JSON}")
  if(CMAKE_MATCH_1)
    set(${PREFIX}_BUILD_DATE ${CMAKE_MATCH_1} PARENT_SCOPE)
  endif()
  
  # 组合完整版本号
  set(_VERSION "${_MAJOR}.${_MINOR}.${_PATCH}")
  set(${PREFIX}_VERSION ${_VERSION} PARENT_SCOPE)
  
  # 设置共享库版本号
  set(_SOVERSION "${_MAJOR}.${_MINOR}")
  set(${PREFIX}_SOVERSION ${_SOVERSION} PARENT_SCOPE)
  
  # 输出版本信息（如果启用详细输出）
  if(NOT ${PREFIX}_VERSION_QUIET)
    message(STATUS "${_NAME} version: ${_VERSION}")
  endif()
endfunction()

