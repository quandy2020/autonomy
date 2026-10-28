include_guard(GLOBAL)

if(TARGET Autolink::Autolink OR TARGET autolink::autolink OR TARGET autolink)
  set(Autolink_FOUND TRUE)
  return()
endif()

find_path(AUTOLINK_INCLUDE_DIR
  NAMES autolink/autolink.hpp
  HINTS ENV AUTOLINK_DIR
  PATH_SUFFIXES include)

find_library(AUTOLINK_LIBRARY
  NAMES autolink
  HINTS ENV AUTOLINK_DIR
  PATH_SUFFIXES lib lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Autolink
  REQUIRED_VARS AUTOLINK_LIBRARY AUTOLINK_INCLUDE_DIR)

if(Autolink_FOUND)
  add_library(autolink::autolink UNKNOWN IMPORTED)
  set_target_properties(autolink::autolink PROPERTIES
    IMPORTED_LOCATION "${AUTOLINK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${AUTOLINK_INCLUDE_DIR}")

  add_library(Autolink::Autolink ALIAS autolink::autolink)
  add_library(autolink::core ALIAS autolink::autolink)
  set(Autolink_INCLUDE_DIRS "${AUTOLINK_INCLUDE_DIR}")
  set(Autolink_LIBRARIES "${AUTOLINK_LIBRARY}")
endif()

mark_as_advanced(AUTOLINK_INCLUDE_DIR AUTOLINK_LIBRARY)
