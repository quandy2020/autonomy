include_guard(GLOBAL)

if(TARGET automsgs::automsgs OR TARGET automsgs::core OR TARGET automsgs)
  set(AutoMsgs_FOUND TRUE)
  return()
endif()

if(NOT TARGET protobuf::libprotobuf AND NOT Protobuf_FOUND)
  find_package(Protobuf QUIET)
endif()

find_path(AUTOMSGS_INCLUDE_DIR
  NAMES automsgs/msgs/sensor_msgs/image.pb.h
  HINTS ENV AUTOMSGS_DIR
  PATH_SUFFIXES include)

find_library(AUTOMSGS_LIBRARY
  NAMES automsgs
  HINTS ENV AUTOMSGS_DIR
  PATH_SUFFIXES lib lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(AutoMsgs
  REQUIRED_VARS AUTOMSGS_LIBRARY AUTOMSGS_INCLUDE_DIR)

if(AutoMsgs_FOUND)
  add_library(automsgs::automsgs UNKNOWN IMPORTED)
  set_target_properties(automsgs::automsgs PROPERTIES
    IMPORTED_LOCATION "${AUTOMSGS_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${AUTOMSGS_INCLUDE_DIR}")

  if(TARGET protobuf::libprotobuf)
    set_property(TARGET automsgs::automsgs APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES protobuf::libprotobuf)
  elseif(Protobuf_LIBRARIES)
    set_property(TARGET automsgs::automsgs APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES "${Protobuf_LIBRARIES}")
  endif()

  add_library(automsgs::core ALIAS automsgs::automsgs)
  set(automsgs_FOUND TRUE)
  set(automsgs_INCLUDE_DIRS "${AUTOMSGS_INCLUDE_DIR}")
  set(automsgs_LIBRARIES "${AUTOMSGS_LIBRARY}")
endif()

mark_as_advanced(AUTOMSGS_INCLUDE_DIR AUTOMSGS_LIBRARY)
