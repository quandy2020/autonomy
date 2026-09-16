# @file FindFCL.cmake
# @brief Optional libfcl — prefer header/library probe (avoid CONFIG side effects).

find_path(FCL_INCLUDE_DIR
  NAMES fcl/fcl.h fcl/narrowphase/collision.h
  PATHS
    /opt/homebrew/include
    /usr/local/include
    /usr/include
)

find_library(FCL_LIBRARY
  NAMES fcl
  PATHS
    /opt/homebrew/lib
    /usr/local/lib
    /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FCL DEFAULT_MSG FCL_LIBRARY FCL_INCLUDE_DIR)

if(FCL_FOUND)
  set(FCL_LIBRARIES ${FCL_LIBRARY})
  set(FCL_INCLUDE_DIRS ${FCL_INCLUDE_DIR})
  if(NOT TARGET FCL::fcl)
    add_library(FCL::fcl UNKNOWN IMPORTED)
    set_target_properties(FCL::fcl PROPERTIES
      IMPORTED_LOCATION "${FCL_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${FCL_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(FCL_INCLUDE_DIR FCL_LIBRARY)
