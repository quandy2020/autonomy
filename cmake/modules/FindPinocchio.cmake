# @file FindPinocchio.cmake
# @brief Optional Pinocchio rigid-body dynamics (manipulation FEATURE pinocchio).
#
# Prefer a manual library probe when the installed pinocchioConfig.cmake would
# FATAL_ERROR on incomplete deps (e.g. missing urdfdom_headers on boards).

# Config export pulls REQUIRED find_dependency(urdfdom_headers); QUIET does not
# suppress that. Only enter CONFIG when headers package is already present.
find_package(urdfdom_headers QUIET CONFIG)
if(urdfdom_headers_FOUND OR urdfdom_headers_DIR)
  find_package(pinocchio QUIET CONFIG
    PATHS /opt/homebrew /usr/local ${CMAKE_INSTALL_PREFIX})
endif()

if(pinocchio_FOUND)
  set(Pinocchio_FOUND TRUE)
  if(NOT TARGET pinocchio::pinocchio AND TARGET pinocchio)
    add_library(pinocchio::pinocchio ALIAS pinocchio)
  endif()
else()
  find_path(PINOCCHIO_INCLUDE_DIR
    NAMES pinocchio/pinocchio.hpp pinocchio/multibody/model.hpp
    PATHS /opt/homebrew/include /usr/local/include /usr/include)
  find_library(PINOCCHIO_LIBRARY
    NAMES pinocchio_default pinocchio
    PATHS /opt/homebrew/lib /usr/local/lib /usr/lib)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Pinocchio DEFAULT_MSG
    PINOCCHIO_LIBRARY PINOCCHIO_INCLUDE_DIR)
  if(Pinocchio_FOUND)
    set(PINOCCHIO_LIBRARIES ${PINOCCHIO_LIBRARY})
    set(PINOCCHIO_INCLUDE_DIRS ${PINOCCHIO_INCLUDE_DIR})
    if(NOT TARGET pinocchio::pinocchio)
      add_library(pinocchio::pinocchio UNKNOWN IMPORTED)
      set_target_properties(pinocchio::pinocchio PROPERTIES
        IMPORTED_LOCATION "${PINOCCHIO_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PINOCCHIO_INCLUDE_DIR}")
    endif()
  endif()
  mark_as_advanced(PINOCCHIO_INCLUDE_DIR PINOCCHIO_LIBRARY)
endif()
