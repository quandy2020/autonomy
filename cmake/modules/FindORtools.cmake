# Optional vendored OR-Tools (TARE-style libortools.so layout).
#
# Set AUTONOMY_ORTOOLS_ROOT to a directory containing include/ and lib/.

if(NOT AUTONOMY_ORTOOLS_ROOT)
  set(ORtools_FOUND FALSE)
  return()
endif()

find_path(ORtools_INCLUDE_DIR
  NAMES ortools/constraint_solver/routing.h
  PATHS "${AUTONOMY_ORTOOLS_ROOT}/include"
  NO_DEFAULT_PATH)

find_library(ORtools_LIBRARY
  NAMES ortools
  PATHS "${AUTONOMY_ORTOOLS_ROOT}/lib"
  NO_DEFAULT_PATH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORtools DEFAULT_MSG
  ORtools_INCLUDE_DIR ORtools_LIBRARY)

if(ORtools_FOUND)
  set(ORtools_INCLUDE_DIRS ${ORtools_INCLUDE_DIR})
  set(ORtools_LIBRARIES ${ORtools_LIBRARY})
endif()
