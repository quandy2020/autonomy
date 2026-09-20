# @file FindPinocchio.cmake
# @brief Optional Pinocchio rigid-body dynamics (manipulation FEATURE pinocchio).
#
# Prefer a manual library probe. The installed pinocchioConfig.cmake pulls
# find_dependency(eigenpy) → Python3 Development.Module / NumPy, which
# FATAL_ERRORs even under QUIET and is unnecessary for C++ RNEA/URDF usage
# on embedded boards.

find_path(PINOCCHIO_INCLUDE_DIR
  NAMES pinocchio/pinocchio.hpp pinocchio/multibody/model.hpp
  PATHS /opt/homebrew/include /usr/local/include /usr/include)

find_library(PINOCCHIO_DEFAULT_LIBRARY
  NAMES pinocchio_default pinocchio
  PATHS /opt/homebrew/lib /usr/local/lib /usr/lib
        /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

find_library(PINOCCHIO_PARSERS_LIBRARY
  NAMES pinocchio_parsers
  PATHS /opt/homebrew/lib /usr/local/lib /usr/lib
        /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

# parsers is required for pinocchio/parsers/urdf.hpp; default alone is not enough.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pinocchio DEFAULT_MSG
  PINOCCHIO_DEFAULT_LIBRARY PINOCCHIO_PARSERS_LIBRARY PINOCCHIO_INCLUDE_DIR)

if(Pinocchio_FOUND)
  set(PINOCCHIO_INCLUDE_DIRS ${PINOCCHIO_INCLUDE_DIR})
  set(PINOCCHIO_LIBRARIES
    ${PINOCCHIO_DEFAULT_LIBRARY}
    ${PINOCCHIO_PARSERS_LIBRARY})

  # Match upstream pinocchioTargets Boost.MPL arity, and disable preprocessed
  # headers so OMPL → Boost.MultiIndex does not fail with arity 30 vs 20.
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
endif()

mark_as_advanced(
  PINOCCHIO_INCLUDE_DIR
  PINOCCHIO_DEFAULT_LIBRARY
  PINOCCHIO_PARSERS_LIBRARY)
