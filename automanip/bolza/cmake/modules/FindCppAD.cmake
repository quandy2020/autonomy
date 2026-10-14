#=============================================================================
# FindCppAD.cmake — CppAD + CppADCodeGen for bolza (cmake/modules/).
#
# Bolza needs <cppad/cg.hpp> (CodeGen layered on CppAD).
#
# Imported target: CppAD::CppAD
# Variables: CppAD_FOUND, CppAD_INCLUDE_DIRS
#=============================================================================
include(FindPackageHandleStandardArgs)

unset(CppAD_FOUND)

# Drop stale cache that points at a prefix without <cppad/cg.hpp>
# (e.g. /usr/include after a partial reconfigure).
if(CppAD_INCLUDE_DIRS AND NOT EXISTS "${CppAD_INCLUDE_DIRS}/cppad/cg.hpp")
  unset(CppAD_INCLUDE_DIRS CACHE)
endif()

find_path(CppAD_INCLUDE_DIRS
  NAMES cppad/cg.hpp
  HINTS
    ENV CppAD_DIR
    ENV CPPAD_DIR
    ENV CPPADCG_DIR
  PATH_SUFFIXES include
  PATHS
    /usr
    /usr/local
    /opt/homebrew
)

find_package_handle_standard_args(CppAD
  REQUIRED_VARS CppAD_INCLUDE_DIRS
)

if(CppAD_FOUND)
  message(STATUS "CppAD: ${CppAD_INCLUDE_DIRS}")
endif()

mark_as_advanced(CppAD_FOUND CppAD_INCLUDE_DIRS)

if(CppAD_FOUND AND NOT TARGET CppAD::CppAD)
  add_library(CppAD::CppAD INTERFACE IMPORTED)
  set_target_properties(CppAD::CppAD PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CppAD_INCLUDE_DIRS}"
  )
endif()
