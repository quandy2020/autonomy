# Copyright 2025 The Openbot Authors (duyongquan)
#
# Find TinyXML2 library (libtinyxml2-dev on Debian/Ubuntu).

find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
find_library(TINYXML2_LIBRARY NAMES tinyxml2)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  TinyXML2
  REQUIRED_VARS TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR
)

if(TinyXML2_FOUND)
  set(TinyXML2_INCLUDE_DIRS ${TINYXML2_INCLUDE_DIR})
  set(TinyXML2_LIBRARIES ${TINYXML2_LIBRARY})
  set(TinyXML2_LIB ${TINYXML2_LIBRARY})

  if(NOT TARGET TinyXML2::TinyXML2)
    add_library(TinyXML2::TinyXML2 UNKNOWN IMPORTED)
    set_target_properties(
      TinyXML2::TinyXML2
      PROPERTIES
        IMPORTED_LOCATION "${TINYXML2_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)
