# @file FindOctomap.cmake
# @brief Optional OctoMap OcTree (manipulation FEATURE octomap).

find_package(octomap QUIET CONFIG
  PATHS /opt/homebrew /usr/local ${CMAKE_INSTALL_PREFIX})

if(octomap_FOUND)
  set(Octomap_FOUND TRUE)
  if(NOT TARGET octomap::octomap)
    if(TARGET octomap)
      add_library(octomap::octomap ALIAS octomap)
    endif()
  endif()
else()
  find_path(OCTOMAP_INCLUDE_DIR
    NAMES octomap/OcTree.h
    PATHS /opt/homebrew/include /usr/local/include /usr/include)
  find_library(OCTOMAP_LIBRARY
    NAMES octomap
    PATHS /opt/homebrew/lib /usr/local/lib /usr/lib)
  find_library(OCTOMATH_LIBRARY
    NAMES octomath
    PATHS /opt/homebrew/lib /usr/local/lib /usr/lib)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Octomap DEFAULT_MSG
    OCTOMAP_LIBRARY OCTOMAP_INCLUDE_DIR)
  if(Octomap_FOUND)
    set(OCTOMAP_LIBRARIES ${OCTOMAP_LIBRARY})
    if(OCTOMATH_LIBRARY)
      list(APPEND OCTOMAP_LIBRARIES ${OCTOMATH_LIBRARY})
    endif()
    set(OCTOMAP_INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIR})
    if(NOT TARGET octomap::octomap)
      add_library(octomap::octomap UNKNOWN IMPORTED)
      set_target_properties(octomap::octomap PROPERTIES
        IMPORTED_LOCATION "${OCTOMAP_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIR}")
    endif()
  endif()
  mark_as_advanced(OCTOMAP_INCLUDE_DIR OCTOMAP_LIBRARY OCTOMATH_LIBRARY)
endif()
