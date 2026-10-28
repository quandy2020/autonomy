include(CMakeParseArguments)

macro(atlas_initialize_project)
  list(PREPEND CMAKE_MODULE_PATH
    "${ATLAS_ROOT}/cmake/modules")
endmacro()

function(atlas_configure_target target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES
    POSITION_INDEPENDENT_CODE ON)

  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(${target} PRIVATE
      -Wall
      -Wextra
      -Wpedantic)
  endif()
endfunction()

function(atlas_binary name)
  set(options)
  set(one_value_arguments)
  set(multi_value_arguments SRCS LIBRARIES)
  cmake_parse_arguments(
    ARG
    "${options}"
    "${one_value_arguments}"
    "${multi_value_arguments}"
    ${ARGN})

  if(NOT ARG_SRCS)
    message(FATAL_ERROR "atlas_binary(${name}) requires SRCS")
  endif()

  add_executable(${name} ${ARG_SRCS})
  atlas_configure_target(${name})
  target_link_libraries(${name} PRIVATE
    atlas_core
    ${ARG_LIBRARIES})
endfunction()

macro(atlas_enable_testing)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
endmacro()

function(atlas_test name)
  set(options)
  set(one_value_arguments)
  set(multi_value_arguments SRCS LIBRARIES)
  cmake_parse_arguments(
    ARG
    "${options}"
    "${one_value_arguments}"
    "${multi_value_arguments}"
    ${ARGN})

  if(NOT ARG_SRCS)
    message(FATAL_ERROR "atlas_test(${name}) requires SRCS")
  endif()

  add_executable(${name} ${ARG_SRCS})
  atlas_configure_target(${name})
  target_link_libraries(${name} PRIVATE
    atlas_core
    GTest::gtest_main
    ${ARG_LIBRARIES})
  gtest_discover_tests(${name})
endfunction()
