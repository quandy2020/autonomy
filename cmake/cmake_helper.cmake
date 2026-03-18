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

if(POLICY CMP0043)
    cmake_policy(SET CMP0043 NEW)
endif()

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

# Determine project compiler.
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(IS_MSVC TRUE)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(IS_GNU TRUE)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(IS_CLANG TRUE)
endif()

# Determine project architecture.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "[ix].?86|amd64|AMD64")
    set(IS_X86 TRUE)
endif()

# Determine project operating system.
string(REGEX MATCH "Linux" IS_LINUX ${CMAKE_SYSTEM_NAME})
string(REGEX MATCH "DragonFly|BSD" IS_BSD ${CMAKE_SYSTEM_NAME})
string(REGEX MATCH "SunOS" IS_SOLARIS ${CMAKE_SYSTEM_NAME})
if(WIN32)
    set(IS_WINDOWS TRUE BOOL INTERNAL)
endif()
if(APPLE)
    set(IS_MACOS TRUE BOOL INTERNAL)
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)
if(CMAKE_BUILD_TYPE_LOWER STREQUAL "debug"
   OR CMAKE_BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
    set(IS_DEBUG TRUE)
endif()

# Enable solution folders.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_TARGETS_ROOT_FOLDER "cmake")
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER
             ${CMAKE_TARGETS_ROOT_FOLDER})
set(AUTONOMY_TARGETS_ROOT_FOLDER "autonomy_targets")
set(AUTONOMY_SRC_ROOT_FOLDER "autonomy_sources")

# This macro will search for source files in a given directory, will add them
# to a source group (folder within a project), and will then return paths to
# each of the found files. The usage of the macro is as follows:
# AUTONOMY_ADD_SOURCE_DIR(
#     <source directory to search>
#     <output variable with found source files>
#     <search expressions such as *.h *.cc>)
macro(AUTONOMY_ADD_SOURCE_DIR SRC_DIR SRC_VAR)
    # Create the list of expressions to be used in the search.
    set(GLOB_EXPRESSIONS "")
    foreach(ARG ${ARGN})
        list(APPEND GLOB_EXPRESSIONS ${SRC_DIR}/${ARG})
    endforeach()
    # Perform the search for the source files.
    file(GLOB ${SRC_VAR} RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
         ${GLOB_EXPRESSIONS})
    # Create the source group.
    string(REPLACE "/" "\\" GROUP_NAME ${SRC_DIR})
    source_group(${GROUP_NAME} FILES ${${SRC_VAR}})
    # Clean-up.
    unset(GLOB_EXPRESSIONS)
    unset(ARG)
    unset(GROUP_NAME)
endmacro(AUTONOMY_ADD_SOURCE_DIR)

# Replacement for the normal add_library() command. The syntax remains the same
# in that the first argument is the target name, and the following arguments
# are the source files to use when building the target.
macro(AUTONOMY_ADD_LIBRARY)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs NAME SRCS PRIVATE_LINK_LIBS PUBLIC_LINK_LIBS)
    cmake_parse_arguments(AUTONOMY_ADD_LIBRARY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_library(${AUTONOMY_ADD_LIBRARY_NAME} STATIC ${AUTONOMY_ADD_LIBRARY_SRCS})
    set_target_properties(${AUTONOMY_ADD_LIBRARY_NAME} PROPERTIES FOLDER
        ${AUTONOMY_TARGETS_ROOT_FOLDER}/${FOLDER_NAME})
    if(CLANG_TIDY_EXE)
        set_target_properties(${AUTONOMY_ADD_LIBRARY_NAME}
            PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-header-filter=.*")
    endif()
    target_link_libraries(${AUTONOMY_ADD_LIBRARY_NAME}
        PRIVATE ${AUTONOMY_ADD_LIBRARY_PRIVATE_LINK_LIBS}
        PUBLIC ${AUTONOMY_ADD_LIBRARY_PUBLIC_LINK_LIBS})
endmacro(AUTONOMY_ADD_LIBRARY)

# Replacement for the normal add_executable() command. The syntax remains the
# same in that the first argument is the target name, and the following
# arguments are the source files to use when building the target.
macro(AUTONOMY_ADD_EXECUTABLE)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs NAME SRCS LINK_LIBS)
    cmake_parse_arguments(AUTONOMY_ADD_EXECUTABLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${AUTONOMY_ADD_EXECUTABLE_NAME} ${AUTONOMY_ADD_EXECUTABLE_SRCS})
    set_target_properties(${AUTONOMY_ADD_EXECUTABLE_NAME} PROPERTIES FOLDER
        ${AUTONOMY_TARGETS_ROOT_FOLDER}/${FOLDER_NAME})
    target_link_libraries(${AUTONOMY_ADD_EXECUTABLE_NAME} ${AUTONOMY_ADD_EXECUTABLE_LINK_LIBS})
    if(VCPKG_BUILD)
        install(TARGETS ${AUTONOMY_ADD_EXECUTABLE_NAME} DESTINATION tools/)
    else()
        install(TARGETS ${AUTONOMY_ADD_EXECUTABLE_NAME} DESTINATION bin/)
    endif()
    if(CLANG_TIDY_EXE)
        set_target_properties(${AUTONOMY_ADD_EXECUTABLE_NAME}
            PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-header-filter=.*")
    endif()
endmacro(AUTONOMY_ADD_EXECUTABLE)

# Wrapper for test executables.
macro(AUTONOMY_ADD_TEST)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs NAME SRCS LINK_LIBS)
    cmake_parse_arguments(AUTONOMY_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(TESTS_ENABLED)
        # ${ARGN} will store the list of link libraries.
        set(AUTONOMY_ADD_TEST_NAME "AUTONOMY_${FOLDER_NAME}_${AUTONOMY_ADD_TEST_NAME}")
        add_executable(${AUTONOMY_ADD_TEST_NAME} ${AUTONOMY_ADD_TEST_SRCS})
        set_target_properties(${AUTONOMY_ADD_TEST_NAME} PROPERTIES FOLDER
            ${AUTONOMY_TARGETS_ROOT_FOLDER}/${FOLDER_NAME})
        if(CLANG_TIDY_EXE)
            set_target_properties(${AUTONOMY_ADD_TEST_NAME}
                PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXE};-header-filter=.*")
        endif()
        target_link_libraries(${AUTONOMY_ADD_TEST_NAME}
            ${AUTONOMY_ADD_TEST_LINK_LIBS}
            GTest::gtest
            GTest::gmock
            GTest::gmock_main)
        add_test("${FOLDER_NAME}/${AUTONOMY_ADD_TEST_NAME}" ${AUTONOMY_ADD_TEST_NAME})
        if(IS_MSVC)
            install(TARGETS ${AUTONOMY_ADD_TEST_NAME} DESTINATION bin/)
        endif()
    endif()
endmacro(AUTONOMY_ADD_TEST)
