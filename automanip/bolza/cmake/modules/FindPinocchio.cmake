#=============================================================================
# FindPinocchio.cmake — Pinocchio for bolza (cmake/modules/).
#
# Prefer a manual library probe. The installed pinocchioConfig.cmake pulls
# find_dependency(eigenpy) → Python3 Development.Module / NumPy, which
# FATAL_ERRORs even under QUIET and is unnecessary for C++ RNEA/URDF usage.
#
# Imported target: pinocchio::pinocchio
# Variables: Pinocchio_FOUND, PINOCCHIO_INCLUDE_DIRS, PINOCCHIO_LIBRARIES
#=============================================================================
include(FindPackageHandleStandardArgs)

find_path(PINOCCHIO_INCLUDE_DIR
  NAMES pinocchio/pinocchio.hpp pinocchio/multibody/model.hpp
  HINTS
    ENV Pinocchio_DIR
    ENV PINOCCHIO_DIR
  PATH_SUFFIXES include
  PATHS
    /opt/homebrew
    /usr/local
    /usr
)

find_library(PINOCCHIO_DEFAULT_LIBRARY
  NAMES pinocchio_default pinocchio
  HINTS
    ENV Pinocchio_DIR
    ENV PINOCCHIO_DIR
  PATH_SUFFIXES lib
  PATHS
    /opt/homebrew
    /usr/local
    /usr
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
)

find_library(PINOCCHIO_PARSERS_LIBRARY
  NAMES pinocchio_parsers
  HINTS
    ENV Pinocchio_DIR
    ENV PINOCCHIO_DIR
  PATH_SUFFIXES lib
  PATHS
    /opt/homebrew
    /usr/local
    /usr
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
)

# parsers is required for pinocchio/parsers/urdf.hpp; default alone is not enough.
find_package_handle_standard_args(Pinocchio DEFAULT_MSG
  PINOCCHIO_DEFAULT_LIBRARY PINOCCHIO_PARSERS_LIBRARY PINOCCHIO_INCLUDE_DIR)

if(Pinocchio_FOUND)
  set(PINOCCHIO_INCLUDE_DIRS ${PINOCCHIO_INCLUDE_DIR})
  set(PINOCCHIO_LIBRARIES
    ${PINOCCHIO_DEFAULT_LIBRARY}
    ${PINOCCHIO_PARSERS_LIBRARY})

  # Match upstream pinocchioTargets Boost.MPL arity, and disable preprocessed
  # headers so Boost.MultiIndex does not fail with arity 30 vs 20.
  set(_PINOCCHIO_COMPILE_DEFS
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_VECTOR_SIZE=30
    BOOST_MPL_LIMIT_LIST_SIZE=30)

  if(NOT TARGET pinocchio::pinocchio)
    add_library(pinocchio::pinocchio UNKNOWN IMPORTED)
    set_target_properties(pinocchio::pinocchio PROPERTIES
      IMPORTED_LOCATION "${PINOCCHIO_DEFAULT_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${PINOCCHIO_INCLUDE_DIR}"
      INTERFACE_COMPILE_DEFINITIONS "${_PINOCCHIO_COMPILE_DEFS}"
      INTERFACE_LINK_LIBRARIES "${PINOCCHIO_PARSERS_LIBRARY}")
  else()
    set_property(TARGET pinocchio::pinocchio APPEND PROPERTY
      INTERFACE_COMPILE_DEFINITIONS ${_PINOCCHIO_COMPILE_DEFS})
  endif()
  unset(_PINOCCHIO_COMPILE_DEFS)

  message(STATUS "Pinocchio: ${PINOCCHIO_DEFAULT_LIBRARY} (includes ${PINOCCHIO_INCLUDE_DIR})")
endif()

mark_as_advanced(
  PINOCCHIO_INCLUDE_DIR
  PINOCCHIO_DEFAULT_LIBRARY
  PINOCCHIO_PARSERS_LIBRARY)
