#=============================================================================
# Copyright (C) 2019, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# FindGLPK.cmake — GLPK for bolza Frank–Wolfe / bilevel (required).
#
# Imported target: GLPK::GLPK
# Variables: GLPK_FOUND, GLPK_INCLUDE_DIRS, GLPK_LIBRARIES
#=============================================================================
include(FindPackageHandleStandardArgs)

unset(GLPK_FOUND)

find_path(GLPK_INCLUDE_DIRS
  NAMES glpk.h
  HINTS /usr/include /usr/local/include
)

find_library(GLPK_LIBRARIES
  NAMES glpk libglpk
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
)

find_package_handle_standard_args(GLPK
  REQUIRED_VARS
    GLPK_INCLUDE_DIRS
    GLPK_LIBRARIES
)

if(GLPK_FOUND)
  message(STATUS "GLPK: ${GLPK_LIBRARIES} (includes ${GLPK_INCLUDE_DIRS})")
endif()

mark_as_advanced(GLPK_FOUND GLPK_INCLUDE_DIRS GLPK_LIBRARIES)

if(GLPK_FOUND AND NOT TARGET GLPK::GLPK)
  add_library(GLPK::GLPK INTERFACE IMPORTED)
  set_target_properties(GLPK::GLPK PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GLPK_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${GLPK_LIBRARIES}"
  )
endif()
