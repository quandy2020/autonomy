# @file FindTRAC_IK.cmake
# @brief Optional TRAC-IK (NLOPT+KDL race) for manipulation FEATURE trac_ik.

find_path(TRAC_IK_INCLUDE_DIR
  NAMES trac_ik/trac_ik.hpp trac_ik.hpp
  PATHS /opt/homebrew/include /usr/local/include /usr/include
        /opt/ros/humble/include /opt/ros/jazzy/include)

find_library(TRAC_IK_LIBRARY
  NAMES trac_ik trac_ik_lib
  PATHS /opt/homebrew/lib /usr/local/lib /usr/lib
        /opt/ros/humble/lib /opt/ros/jazzy/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TRAC_IK DEFAULT_MSG
  TRAC_IK_LIBRARY TRAC_IK_INCLUDE_DIR)

if(TRAC_IK_FOUND)
  set(TRAC_IK_LIBRARIES ${TRAC_IK_LIBRARY})
  set(TRAC_IK_INCLUDE_DIRS ${TRAC_IK_INCLUDE_DIR})
  if(NOT TARGET TRAC_IK::TRAC_IK)
    add_library(TRAC_IK::TRAC_IK UNKNOWN IMPORTED)
    set_target_properties(TRAC_IK::TRAC_IK PROPERTIES
      IMPORTED_LOCATION "${TRAC_IK_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${TRAC_IK_INCLUDE_DIR}")
  endif()
endif()
mark_as_advanced(TRAC_IK_INCLUDE_DIR TRAC_IK_LIBRARY)
