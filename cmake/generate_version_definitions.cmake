# Copyright 2024 The Openbot Authors (duyongquan)
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

find_package(Git REQUIRED)

if (DEFINED GIT_COMMIT_ID OR DEFINED GIT_COMMIT_DATE)
    message(STATUS "Using custom-defined GIT_COMMIT_ID (${GIT_COMMIT_ID}) "
            "and GIT_COMMIT_DATE (${GIT_COMMIT_DATE})")
elseif(Git_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    message(STATUS "Found Git: ${GIT_EXECUTABLE}")

    execute_process(COMMAND
        "${GIT_EXECUTABLE}" rev-parse --short HEAD
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_ID
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

     # 获取最近提交的作者名字
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%an
        OUTPUT_VARIABLE GIT_COMMIT_AUTHOR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%ad --date=short
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

    # 获取最近提交的邮箱
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:%ae
        OUTPUT_VARIABLE GIT_COMMIT_EMAIL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
        
    # 获取最近提交的日期
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --pretty=format:"%ad" --date=format:"%Y-%m-%d %H:%M:%S"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    # 获取版本描述
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    # 获取分支信息
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    # 输出获取到的Git信息
    message(STATUS "Git Author: ${GIT_COMMIT_AUTHOR}")
    message(STATUS "Git Email: ${GIT_COMMIT_EMAIL}")
    message(STATUS "Git Date: ${GIT_COMMIT_DATE}")
    message(STATUS "Git Version: ${GIT_VERSION}")
    message(STATUS "Git Branch: ${GIT_BRANCH}")

    # Re-generate version.cc if the git index changes.
    set_property(
        DIRECTORY APPEND 
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${PROJECT_SOURCE_DIR}/.git/index"
    )
else()
    message(WARNING "Git not found. Using default values for version info.")
    set(GIT_BRANCH "unknown")
    set(GIT_COMMIT_ID "Unknown")
    set(GIT_COMMIT_DATE "Unknown")
    set(GIT_COMMIT_AUTHOR "unknown")
    set(GIT_COMMIT_EMAIL "unknown")
    set(GIT_VERSION "unknown")
    set(BUILD_TIMESTAMP "unknown")
    set(BUILD_HOST "unknown")
    set(BUILD_USER "unknown")
    
endif()

# 获取构建时间戳
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)

# 获取构建主机和用户信息
if(UNIX)
    execute_process(
        COMMAND hostname
        OUTPUT_VARIABLE BUILD_HOST
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND whoami
        OUTPUT_VARIABLE BUILD_USER
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
elseif(WIN32)
    execute_process(
        COMMAND hostname
        OUTPUT_VARIABLE BUILD_HOST
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND whoami
        OUTPUT_VARIABLE BUILD_USER
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

# 获取系统信息
if(CMAKE_SYSTEM_NAME)
    set(SYSTEM_NAME ${CMAKE_SYSTEM_NAME})
else()
    set(SYSTEM_NAME "unknown")
endif()

if(CMAKE_SYSTEM_PROCESSOR)
    set(SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
else()
    set(SYSTEM_PROCESSOR "unknown")
endif()

if(CMAKE_SYSTEM_VERSION)
    set(SYSTEM_VERSION ${CMAKE_SYSTEM_VERSION})
else()
    set(SYSTEM_VERSION "unknown")
endif()

if(CMAKE_CXX_COMPILER_ID)
    set(COMPILER_ID ${CMAKE_CXX_COMPILER_ID})
else()
    set(COMPILER_ID "unknown")
endif()

if(CMAKE_CXX_COMPILER_VERSION)
    set(COMPILER_VERSION ${CMAKE_CXX_COMPILER_VERSION})
else()
    set(COMPILER_VERSION "unknown")
endif()

# 输出系统信息
message(STATUS "Build Timestamp: ${BUILD_TIMESTAMP}")
message(STATUS "Build Host: ${BUILD_HOST}")
message(STATUS "Build User: ${BUILD_USER}")
message(STATUS "System Name: ${SYSTEM_NAME}")
message(STATUS "System Processor: ${SYSTEM_PROCESSOR}")
message(STATUS "System Version: ${SYSTEM_VERSION}")
message(STATUS "Compiler ID: ${COMPILER_ID}")
message(STATUS "Compiler Version: ${COMPILER_VERSION}")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/autonomy/common/version.cpp.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/autonomy/common/version.cpp"
               @ONLY)
