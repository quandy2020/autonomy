# Copyright 2026 The Openbot Authors
# Adapted from Cartographer cmake/functions.cmake (Apache-2.0).
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

include(CMakeParseArguments)

macro(_argus_parse_arguments ARGS)
  set(OPTIONS)
  set(ONE_VALUE_ARG)
  set(MULTI_VALUE_ARGS SRCS)
  cmake_parse_arguments(ARG
    "${OPTIONS}" "${ONE_VALUE_ARG}" "${MULTI_VALUE_ARGS}" ${ARGS})
endmacro(_argus_parse_arguments)

macro(_argus_common_compile_stuff)
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${ARGUS_CXX_FLAGS}")
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "${TARGET_COMPILE_FLAGS}")
  target_link_libraries(${NAME} PUBLIC ${PROJECT_NAME})
endmacro(_argus_common_compile_stuff)

# Create variable VAR_NAME='FLAG'. If already set, FLAG is appended.
function(argus_add_flag VAR_NAME FLAG)
  if(${VAR_NAME})
    set(${VAR_NAME} "${${VAR_NAME}} ${FLAG}" PARENT_SCOPE)
  else()
    set(${VAR_NAME} "${FLAG}" PARENT_SCOPE)
  endif()
endfunction()

macro(argus_initialize_project)
  if(ARGUS_CMAKE_DIR)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${ARGUS_CMAKE_DIR}/modules)
  else()
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
  endif()

  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(WIN32)
    # Keep defaults; MSVC flags set on the library target if needed.
  else()
    set(ARGUS_CXX_FLAGS "-pthread -fPIC ${ARGUS_CXX_FLAGS}")
    argus_add_flag(ARGUS_CXX_FLAGS "-Wall")
    argus_add_flag(ARGUS_CXX_FLAGS "-Wpedantic")
    argus_add_flag(ARGUS_CXX_FLAGS "-Wno-deprecated-declarations")
    argus_add_flag(ARGUS_CXX_FLAGS "-Werror=format-security")
    argus_add_flag(ARGUS_CXX_FLAGS "-Werror=return-type")
    argus_add_flag(ARGUS_CXX_FLAGS "-Werror=switch")
    argus_add_flag(ARGUS_CXX_FLAGS "-Werror=uninitialized")

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR
       CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
      argus_add_flag(ARGUS_CXX_FLAGS "-Wthread-safety")
    endif()

    if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
      set(CMAKE_BUILD_TYPE Release)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
      argus_add_flag(ARGUS_CXX_FLAGS "-O3 -DNDEBUG")
    elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
      argus_add_flag(ARGUS_CXX_FLAGS "-O3 -g -DNDEBUG")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
      message(STATUS "Argus Debug build (expect slower runtime).")
      argus_add_flag(ARGUS_CXX_FLAGS "-g")
    elseif(CMAKE_BUILD_TYPE STREQUAL "None")
      message(WARNING "Building with CMAKE_BUILD_TYPE=None; "
          "ensure CFLAGS/CXXFLAGS are set as needed.")
    else()
      message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
    endif()

    message(STATUS "Argus build type: ${CMAKE_BUILD_TYPE}")

    # Rerun CMake when sources are added/removed (Cartographer pattern).
    set(LIST_FILES_CMD
      "find ${PROJECT_SOURCE_DIR}/argus ${PROJECT_SOURCE_DIR}/apps ${PROJECT_SOURCE_DIR}/test ${PROJECT_SOURCE_DIR}/cmake ${PROJECT_SOURCE_DIR}/config -type f 2>/dev/null | sort | sed 's/^/#/'")
    set(FILES_LIST_PATH "${PROJECT_BINARY_DIR}/AllFiles.cmake")
    set(DETECT_CHANGES_CMD "bash" "-c"
      "${LIST_FILES_CMD} | diff -N -q ${FILES_LIST_PATH} - || ${LIST_FILES_CMD} > ${FILES_LIST_PATH}")
    add_custom_target(${PROJECT_NAME}_detect_changes ALL
      COMMAND ${DETECT_CHANGES_CMD}
      VERBATIM
    )
    if(NOT EXISTS ${FILES_LIST_PATH})
      execute_process(COMMAND ${DETECT_CHANGES_CMD})
    endif()
    include(${FILES_LIST_PATH})
  endif()
endmacro()

macro(argus_enable_testing)
  enable_testing()
endmacro()

# Unit test executable linked against the Argus library.
function(argus_test NAME)
  _argus_parse_arguments("${ARGN}")
  if(NOT ARG_SRCS)
    message(FATAL_ERROR "argus_test(${NAME}): SRCS required")
  endif()
  add_executable(${NAME} ${ARG_SRCS})
  set(NAME ${NAME})
  _argus_common_compile_stuff()
  add_test(NAME ${NAME} COMMAND ${NAME}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endfunction()

# Installable binary linked against the Argus library.
function(argus_binary NAME)
  _argus_parse_arguments("${ARGN}")
  if(NOT ARG_SRCS)
    message(FATAL_ERROR "argus_binary(${NAME}): SRCS required")
  endif()
  add_executable(${NAME} ${ARG_SRCS})
  set(NAME ${NAME})
  _argus_common_compile_stuff()
  if(ARGUS_BUILD_INSTALL)
    install(TARGETS "${NAME}" RUNTIME DESTINATION bin)
  endif()
endfunction()
