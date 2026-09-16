# @file FindRuckig.cmake
# @brief Optional Ruckig trajectory smoother.

find_package(ruckig QUIET CONFIG
  PATHS /opt/homebrew /usr/local ${CMAKE_INSTALL_PREFIX})

if(ruckig_FOUND)
  set(Ruckig_FOUND TRUE)
else()
  find_path(RUCKIG_INCLUDE_DIR
    NAMES ruckig/ruckig.hpp
    PATHS /opt/homebrew/include /usr/local/include /usr/include)
  find_library(RUCKIG_LIBRARY
    NAMES ruckig
    PATHS /opt/homebrew/lib /usr/local/lib /usr/lib)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Ruckig DEFAULT_MSG
    RUCKIG_LIBRARY RUCKIG_INCLUDE_DIR)
  if(Ruckig_FOUND)
    set(RUCKIG_LIBRARIES ${RUCKIG_LIBRARY})
    set(RUCKIG_INCLUDE_DIRS ${RUCKIG_INCLUDE_DIR})
    if(NOT TARGET ruckig::ruckig)
      add_library(ruckig::ruckig UNKNOWN IMPORTED)
      set_target_properties(ruckig::ruckig PROPERTIES
        IMPORTED_LOCATION "${RUCKIG_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${RUCKIG_INCLUDE_DIR}")
    endif()
  endif()
  mark_as_advanced(RUCKIG_INCLUDE_DIR RUCKIG_LIBRARY)
endif()
