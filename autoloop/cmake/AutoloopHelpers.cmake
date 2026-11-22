# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

function(autoloop_library name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDENCIES;INCLUDES" ${ARGN})
  add_library(${name} ${ARG_SOURCES})
  target_include_directories(${name}
    PUBLIC
      ${AUTOLOOP_SRC_ROOT}
      ${AUTOLOOP_GEN_DIR}
      ${ARG_INCLUDES})
  target_compile_features(${name} PUBLIC cxx_std_20)
  if(ARG_DEPENDENCIES)
    target_link_libraries(${name} PUBLIC ${ARG_DEPENDENCIES})
  endif()
endfunction()

function(autoloop_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPENDENCIES" ${ARGN})
  if(NOT BUILD_TESTING)
    return()
  endif()
  if(NOT TARGET GTest::gtest_main)
    return()
  endif()
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_DEPENDENCIES} GTest::gtest_main)
  target_include_directories(${name} PRIVATE ${AUTOLOOP_SRC_ROOT} ${AUTOLOOP_GEN_DIR})
  add_test(NAME ${name} COMMAND ${name})
endfunction()
