# @file FindVHACD.cmake
# @brief Optional VHACD convex decomposition (manipulation FEATURE vhacd).

find_path(VHACD_INCLUDE_DIR
  NAMES VHACD.h vhacd/VHACD.h VHACD/VHACD.h
  PATHS /opt/homebrew/include /usr/local/include /usr/include
        ${CMAKE_INSTALL_PREFIX}/include)
find_library(VHACD_LIBRARY
  NAMES VHACD vhacd
  PATHS /opt/homebrew/lib /usr/local/lib /usr/lib
        ${CMAKE_INSTALL_PREFIX}/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VHACD DEFAULT_MSG
  VHACD_LIBRARY VHACD_INCLUDE_DIR)

if(VHACD_FOUND)
  set(VHACD_LIBRARIES ${VHACD_LIBRARY})
  set(VHACD_INCLUDE_DIRS ${VHACD_INCLUDE_DIR})
  if(NOT TARGET VHACD::VHACD)
    add_library(VHACD::VHACD UNKNOWN IMPORTED)
    set_target_properties(VHACD::VHACD PROPERTIES
      IMPORTED_LOCATION "${VHACD_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${VHACD_INCLUDE_DIR}")
  endif()
endif()
mark_as_advanced(VHACD_INCLUDE_DIR VHACD_LIBRARY)
