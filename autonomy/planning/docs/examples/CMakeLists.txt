cmake_minimum_required(VERSION 3.20)
project(fields2cover_documentation_example LANGUAGES CXX)

get_filename_component(_default_source_root "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(AUTONOMY_SOURCE_ROOT "${_default_source_root}" CACHE PATH "Autonomy source root")
get_filename_component(_default_workspace "${AUTONOMY_SOURCE_ROOT}/../.." ABSOLUTE)
set(AUTONOMY_BUILD_ROOT "${_default_workspace}/build/autonomy" CACHE PATH "Existing autonomy build directory")

if(NOT EXISTS "${AUTONOMY_SOURCE_ROOT}/autonomy/planning/planner/fields2cover/fields2cover.hpp")
  message(FATAL_ERROR "AUTONOMY_SOURCE_ROOT must contain autonomy/planning/planner/fields2cover")
endif()
find_library(AUTONOMY_PLANNING_LIBRARY NAMES autonomy_planning
  PATHS "${AUTONOMY_BUILD_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(AUTONOMY_COMMON_LIBRARY NAMES autonomy_common
  PATHS "${AUTONOMY_BUILD_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_package(Eigen3 REQUIRED)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/COLCON_IGNORE" "")
add_executable(coverage_demo coverage_demo.cc)
target_compile_features(coverage_demo PRIVATE cxx_std_20)
target_include_directories(coverage_demo PRIVATE
  "${AUTONOMY_SOURCE_ROOT}" "${AUTONOMY_BUILD_ROOT}")
target_link_libraries(coverage_demo PRIVATE
  "${AUTONOMY_PLANNING_LIBRARY}" "${AUTONOMY_COMMON_LIBRARY}" Eigen3::Eigen)
if(UNIX AND NOT APPLE)
  target_link_options(coverage_demo PRIVATE "LINKER:-rpath-link,/usr/local/lib")
endif()
set_target_properties(coverage_demo PROPERTIES BUILD_RPATH "${AUTONOMY_BUILD_ROOT}/lib;/usr/local/lib")
