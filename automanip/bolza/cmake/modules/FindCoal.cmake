#=============================================================================
# FindCoal.cmake — Coal (hpp-fcl successor) for bolza pinocchio collision.
#
# Imported target: coal::coal
# Variables: Coal_FOUND
#=============================================================================
include(FindPackageHandleStandardArgs)

if(TARGET coal::coal)
  set(Coal_FOUND TRUE)
  return()
endif()

find_package(coal QUIET CONFIG)
if(coal_FOUND AND TARGET coal::coal)
  set(Coal_FOUND TRUE)
  message(STATUS "Coal: coal::coal (CONFIG)")
  return()
endif()

# Legacy package name still ships coal::coal / hpp-fcl::hpp-fcl.
find_package(hpp-fcl QUIET CONFIG)
if(TARGET coal::coal)
  set(Coal_FOUND TRUE)
  message(STATUS "Coal: coal::coal (via hpp-fcl CONFIG)")
  return()
endif()
if(TARGET hpp-fcl::hpp-fcl AND NOT TARGET coal::coal)
  add_library(coal::coal INTERFACE IMPORTED)
  set_target_properties(coal::coal PROPERTIES
    INTERFACE_LINK_LIBRARIES hpp-fcl::hpp-fcl)
  set(Coal_FOUND TRUE)
  message(STATUS "Coal: hpp-fcl::hpp-fcl → coal::coal")
  return()
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(PC_COAL QUIET coal)
endif()

find_path(COAL_INCLUDE_DIR
  NAMES coal/collision_data.h hpp/fcl/collision_data.h
  HINTS ${PC_COAL_INCLUDE_DIRS}
  PATH_SUFFIXES include
  PATHS /usr /usr/local /opt/homebrew)

find_library(COAL_LIBRARY
  NAMES coal hpp-fcl
  HINTS ${PC_COAL_LIBRARY_DIRS}
  PATH_SUFFIXES lib
  PATHS /usr /usr/local /opt/homebrew
        /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)

find_package_handle_standard_args(Coal DEFAULT_MSG COAL_LIBRARY COAL_INCLUDE_DIR)

if(Coal_FOUND AND NOT TARGET coal::coal)
  add_library(coal::coal UNKNOWN IMPORTED)
  set_target_properties(coal::coal PROPERTIES
    IMPORTED_LOCATION "${COAL_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${COAL_INCLUDE_DIR}")
  message(STATUS "Coal: ${COAL_LIBRARY}")
endif()

mark_as_advanced(COAL_INCLUDE_DIR COAL_LIBRARY)
