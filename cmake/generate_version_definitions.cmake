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
    execute_process(COMMAND
        "${GIT_EXECUTABLE}" rev-parse --short HEAD
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_ID
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

    execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%ad --date=short
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

    # Re-generate version.cc if the git index changes.
    set_property(
        DIRECTORY APPEND 
        PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${PROJECT_SOURCE_DIR}/.git/index"
    )
else()
    set(GIT_COMMIT_ID "Unknown")
    set(GIT_COMMIT_DATE "Unknown")
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/autonomy/common/version.cpp.in"
               "${CMAKE_CURRENT_SOURCE_DIR}/autonomy/common/version.cpp")
