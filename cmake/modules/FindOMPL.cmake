# @file FindOMPL.cmake
# @brief Optional OMPL — header/library probe only (avoid omplConfig→Boost).
#
# Homebrew lays headers under include/ompl-1.7/ompl/...
# OMPL bottles often need libboost_system at runtime; if missing (Boost≥1.89
# header-only system), we disable OMPL_FOUND so the stub planner is used.

find_path(OMPL_INCLUDE_DIR
  NAMES ompl/config.h
  PATHS
    /opt/homebrew/include
    /usr/local/include
    /usr/include
  PATH_SUFFIXES
    ompl-1.7
    ompl-1.6
    ompl-1.5
)

find_library(OMPL_LIBRARY
  NAMES ompl
  PATHS
    /opt/homebrew/lib
    /usr/local/lib
    /usr/lib
)

# Companion Boost.System shared lib (OMPL dylib dependency on macOS bottles).
find_library(OMPL_BOOST_SYSTEM_LIBRARY
  NAMES boost_system
  PATHS
    /opt/homebrew/lib
    /opt/homebrew/opt/boost/lib
    /opt/homebrew/Cellar/boost/1.88.0/lib
    /usr/local/lib
    /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OMPL DEFAULT_MSG OMPL_LIBRARY OMPL_INCLUDE_DIR)

if(OMPL_FOUND)
  if(APPLE AND NOT OMPL_BOOST_SYSTEM_LIBRARY)
    message(STATUS
      "OMPL library found but libboost_system missing; "
      "disabling OMPL adapter (use stub / reinstall matching boost+ompl)")
    set(OMPL_FOUND FALSE)
  else()
    set(OMPL_LIBRARIES ${OMPL_LIBRARY})
    if(OMPL_BOOST_SYSTEM_LIBRARY)
      list(APPEND OMPL_LIBRARIES ${OMPL_BOOST_SYSTEM_LIBRARY})
    endif()
    set(OMPL_INCLUDE_DIRS ${OMPL_INCLUDE_DIR})
    if(NOT TARGET OMPL::OMPL)
      add_library(OMPL::OMPL UNKNOWN IMPORTED)
      set_target_properties(OMPL::OMPL PROPERTIES
        IMPORTED_LOCATION "${OMPL_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OMPL_INCLUDE_DIR}")
      if(OMPL_BOOST_SYSTEM_LIBRARY)
        set_property(TARGET OMPL::OMPL APPEND PROPERTY
          INTERFACE_LINK_LIBRARIES "${OMPL_BOOST_SYSTEM_LIBRARY}")
      endif()
    endif()
  endif()
endif()

mark_as_advanced(OMPL_INCLUDE_DIR OMPL_LIBRARY OMPL_BOOST_SYSTEM_LIBRARY)
