# Copyright 2026 The Openbot Authors
# Adapted from Cartographer / autolink FindEigen3 (Apache-2.0).
#
# find_package(Eigen3 REQUIRED)
#   → Eigen3::Eigen (INTERFACE), EIGEN3_INCLUDE_DIR, EIGEN3_FOUND

include(FindPackageHandleStandardArgs)

if(TARGET Eigen3::Eigen)
  get_target_property(_argus_eigen_inc Eigen3::Eigen
    INTERFACE_INCLUDE_DIRECTORIES)
  if(_argus_eigen_inc)
    set(EIGEN3_INCLUDE_DIR "${_argus_eigen_inc}")
  endif()
  set(EIGEN3_FOUND TRUE)
  set(Eigen3_FOUND TRUE)
  find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR)
  return()
endif()

# Prefer the upstream Config package (avoids recursion via NO_MODULE).
find_package(Eigen3 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  get_target_property(_argus_eigen_inc Eigen3::Eigen
    INTERFACE_INCLUDE_DIRECTORIES)
  if(_argus_eigen_inc)
    set(EIGEN3_INCLUDE_DIR "${_argus_eigen_inc}")
  endif()
  set(EIGEN3_FOUND TRUE)
  set(Eigen3_FOUND TRUE)
  find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR)
  mark_as_advanced(EIGEN3_INCLUDE_DIR)
  return()
endif()

set(_ARGUS_EIGEN3_HINTS
  "$ENV{EIGEN3_ROOT}"
  "$ENV{EIGEN_ROOT}"
  /opt/homebrew/include/eigen3
  /usr/local/include/eigen3
  /usr/include/eigen3
  /usr/include/eigen3
)

find_path(EIGEN3_INCLUDE_DIR
  NAMES Eigen/Core
  HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
  PATHS ${_ARGUS_EIGEN3_HINTS}
  PATH_SUFFIXES include/eigen3 eigen3 include
)

if(EIGEN3_INCLUDE_DIR)
  set(EIGEN3_FOUND TRUE)
  set(Eigen3_FOUND TRUE)
  set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
  if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
  endif()
endif()

find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR)
mark_as_advanced(EIGEN3_INCLUDE_DIR)
unset(_ARGUS_EIGEN3_HINTS)
unset(_argus_eigen_inc)
