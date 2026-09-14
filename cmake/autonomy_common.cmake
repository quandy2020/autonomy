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

# @file autonomy_common.cmake
# @brief Global compile flags, gtest wrapper, and GMock discovery.
#
# Loaded by autonomy_configure_project() via autonomy_initialize_project().

# @brief Add a gtest executable and register_test.
# @param NAME Target name (often a dotted path).
# @param ARG_SRC Single .cpp source file.
# @param LINK_TARGET Domain library under test.
function(autonomy_test NAME ARG_SRC LINK_TARGET)
  add_executable(${NAME} ${ARG_SRC})
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${AUTONOMY_CXX_FLAGS}")
  target_include_directories(${NAME} PRIVATE
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_BINARY_DIR}")

  # Set RPATH properties to avoid warnings - use $ORIGIN for relative paths
  set_target_properties(${NAME} PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH FALSE
    INSTALL_RPATH "\$ORIGIN/../lib:${CMAKE_INSTALL_PREFIX}/lib"
    BUILD_RPATH "\$ORIGIN:\$ORIGIN/../lib:${CMAKE_BINARY_DIR}/lib"
    SKIP_BUILD_RPATH FALSE
  )

  # Make sure that gmock always includes the correct gtest/gtest.h.
  target_include_directories("${NAME}" SYSTEM PRIVATE
    "${GMOCK_INCLUDE_DIRS}")
  target_link_libraries("${NAME}" PUBLIC ${GMOCK_LIBRARIES})
  target_link_libraries("${NAME}" PUBLIC ${LINK_TARGET})
  target_compile_options(${NAME} PRIVATE -Wno-error=missing-braces)

  add_test(NAME ${NAME} COMMAND $<TARGET_FILE:${NAME}>)
endfunction()

# @brief Append a compile flag to @c VAR_NAME (space-separated).
# @param VAR_NAME Parent-scope variable name.
# @param FLAG Flag string to append.
function(autonomy_add_flag VAR_NAME FLAG)
  if (${VAR_NAME})
    set(${VAR_NAME} "${${VAR_NAME}} ${FLAG}" PARENT_SCOPE)
  else()
    set(${VAR_NAME} "${FLAG}" PARENT_SCOPE)
  endif()
endfunction()

# @brief Set AUTONOMY_CXX_FLAGS, CMAKE_MODULE_PATH, and Debug-build policy.
macro(autonomy_initialize_project)
  if(AUTONOMY_CMAKE_DIR)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${AUTONOMY_CMAKE_DIR}/modules)
  else()
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
  endif()

  if(WIN32)
    # TODO turn on equivalent warnings on Windows
  else()
    set(AUTONOMY_CXX_FLAGS "-pthread -fPIC ${AUTONOMY_CXX_FLAGS}")

    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Wall")
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Wpedantic")

    # Turn some warnings into errors.
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Werror=format-security")
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Werror=missing-braces")
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Werror=reorder")
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Werror=return-type")
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Werror=switch")
    autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Werror=uninitialized")

    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "AppleClang")
      autonomy_add_flag(AUTONOMY_CXX_FLAGS "-Wthread-safety")
    endif()

    if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
      autonomy_add_flag(AUTONOMY_CXX_FLAGS "-O3 -DNDEBUG")
    elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
      autonomy_add_flag(AUTONOMY_CXX_FLAGS "-O3 -g -DNDEBUG")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
      if(FORCE_DEBUG_BUILD)
        message(WARNING "Building in Debug mode, expect very slow performance.")
        autonomy_add_flag(AUTONOMY_CXX_FLAGS "-g")
      else()
        message(FATAL_ERROR
          "Compiling in Debug mode is not supported and can cause severely degraded performance. "
          "You should change the build type to Release. If you want to build in Debug mode anyway, "
          "call CMake with -DFORCE_DEBUG_BUILD=True"
        )
      endif()
    # Support for Debian packaging CMAKE_BUILD_TYPE
    elseif(CMAKE_BUILD_TYPE STREQUAL "None")
      message(WARNING "Building with CMAKE_BUILD_TYPE None, "
          "please make sure you have set CFLAGS and CXXFLAGS according to your needs.")
    else()
      message(FATAL_ERROR "Unknown CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
    endif()

    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
  endif()
endmacro()

# @brief enable_testing() and find_package(GMock).
macro(autonomy_enable_testing)
  enable_testing()
  find_package(GMock REQUIRED)
endmacro()
