# @file FindPinocchio.cmake
# @brief Optional Pinocchio rigid-body dynamics (manipulation FEATURE pinocchio).
#
# Prefer a manual library probe. The installed pinocchioConfig.cmake pulls
# find_dependency(eigenpy) → Python3 Development.Module / NumPy, which
# FATAL_ERRORs even under QUIET and is unnecessary for C++ RNEA/URDF usage
# on embedded boards.

find_path(PINOCCHIO_INCLUDE_DIR
  NAMES pinocchio/multibody/model.hpp pinocchio/pinocchio.hpp pinocchio/fwd.hpp
  PATHS /opt/ros/humble/include /opt/homebrew/include /usr/local/include /usr/include)

# Drop stale cache entries when the include tree was removed (libs may remain).
if(PINOCCHIO_INCLUDE_DIR AND NOT EXISTS
    "${PINOCCHIO_INCLUDE_DIR}/pinocchio/algorithm/rnea.hpp"
   AND NOT EXISTS
    "${PINOCCHIO_INCLUDE_DIR}/pinocchio/deprecated/pinocchio/algorithm/rnea.hpp")
  unset(PINOCCHIO_INCLUDE_DIR CACHE)
  unset(PINOCCHIO_INCLUDE_DIR)
  find_path(PINOCCHIO_INCLUDE_DIR
    NAMES pinocchio/algorithm/rnea.hpp
          pinocchio/multibody/model.hpp
          pinocchio/fwd.hpp
    PATHS /opt/ros/humble/include /opt/homebrew/include /usr/local/include /usr/include)
endif()

find_library(PINOCCHIO_DEFAULT_LIBRARY
  NAMES pinocchio_default pinocchio
  PATHS /opt/ros/humble/lib /opt/homebrew/lib /usr/local/lib /usr/lib
        /opt/ros/humble/lib/aarch64-linux-gnu /opt/ros/humble/lib/x86_64-linux-gnu
        /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

find_library(PINOCCHIO_PARSERS_LIBRARY
  NAMES pinocchio_parsers
  PATHS /opt/ros/humble/lib /opt/homebrew/lib /usr/local/lib /usr/lib
        /opt/ros/humble/lib/aarch64-linux-gnu /opt/ros/humble/lib/x86_64-linux-gnu
        /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

# parsers is required for pinocchio/parsers/urdf.hpp; default alone is not enough.
# Require RNEA header so AUTOMANIP_HAS_PINOCCHIO is not set against a lib-only install.
set(PINOCCHIO_RNEA_HEADER "")
if(PINOCCHIO_INCLUDE_DIR)
  if(EXISTS "${PINOCCHIO_INCLUDE_DIR}/pinocchio/algorithm/rnea.hpp")
    set(PINOCCHIO_RNEA_HEADER
      "${PINOCCHIO_INCLUDE_DIR}/pinocchio/algorithm/rnea.hpp")
  elseif(EXISTS
      "${PINOCCHIO_INCLUDE_DIR}/pinocchio/deprecated/pinocchio/algorithm/rnea.hpp")
    set(PINOCCHIO_RNEA_HEADER
      "${PINOCCHIO_INCLUDE_DIR}/pinocchio/deprecated/pinocchio/algorithm/rnea.hpp")
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pinocchio DEFAULT_MSG
  PINOCCHIO_DEFAULT_LIBRARY PINOCCHIO_PARSERS_LIBRARY
  PINOCCHIO_INCLUDE_DIR PINOCCHIO_RNEA_HEADER)

if(Pinocchio_FOUND)
  set(PINOCCHIO_INCLUDE_DIRS ${PINOCCHIO_INCLUDE_DIR})
  set(PINOCCHIO_LIBRARIES
    ${PINOCCHIO_DEFAULT_LIBRARY}
    ${PINOCCHIO_PARSERS_LIBRARY})

  # Match upstream pinocchioTargets Boost.MPL arity, and disable preprocessed
  # headers so OMPL → Boost.MultiIndex does not fail with arity 30 vs 20.
  find_library(PINOCCHIO_COLLISION_LIBRARY
    NAMES pinocchio_collision
    PATHS /opt/ros/humble/lib /opt/homebrew/lib /usr/local/lib /usr/lib
          /opt/ros/humble/lib/aarch64-linux-gnu /opt/ros/humble/lib/x86_64-linux-gnu
          /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

  set(_PINOCCHIO_COMPILE_DEFS
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_VECTOR_SIZE=30
    BOOST_MPL_LIMIT_LIST_SIZE=30
    BOOST_FUSION_INVOKE_MAX_ARITY=12
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
    PINOCCHIO_WITH_COLLISION
    PINOCCHIO_WITH_HPP_FCL)

  set(_pinocchio_includes "${PINOCCHIO_INCLUDE_DIR}")
  # Pinocchio 4 moved model/data/geometry headers under include/pinocchio/deprecated.
  if(NOT EXISTS "${PINOCCHIO_INCLUDE_DIR}/pinocchio/multibody/model.hpp"
     AND EXISTS "${PINOCCHIO_INCLUDE_DIR}/pinocchio/deprecated/pinocchio/multibody/model.hpp")
    list(APPEND _pinocchio_includes
      "${PINOCCHIO_INCLUDE_DIR}/pinocchio/deprecated")
  endif()

  if(NOT TARGET pinocchio::pinocchio)
    add_library(pinocchio::pinocchio UNKNOWN IMPORTED)
    set_target_properties(pinocchio::pinocchio PROPERTIES
      IMPORTED_LOCATION "${PINOCCHIO_DEFAULT_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${_pinocchio_includes}"
      INTERFACE_COMPILE_DEFINITIONS "${_PINOCCHIO_COMPILE_DEFS}"
      INTERFACE_LINK_LIBRARIES "${PINOCCHIO_PARSERS_LIBRARY}")
    if(PINOCCHIO_COLLISION_LIBRARY)
      set_property(TARGET pinocchio::pinocchio APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${PINOCCHIO_COLLISION_LIBRARY}")
    endif()
  else()
    set_property(TARGET pinocchio::pinocchio APPEND PROPERTY
      INTERFACE_COMPILE_DEFINITIONS ${_PINOCCHIO_COMPILE_DEFS})
    if(PINOCCHIO_COLLISION_LIBRARY)
      set_property(TARGET pinocchio::pinocchio APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${PINOCCHIO_COLLISION_LIBRARY}")
    endif()
  endif()
  unset(_PINOCCHIO_COMPILE_DEFS)
  unset(_pinocchio_includes)
endif()

mark_as_advanced(
  PINOCCHIO_INCLUDE_DIR
  PINOCCHIO_DEFAULT_LIBRARY
  PINOCCHIO_PARSERS_LIBRARY)
