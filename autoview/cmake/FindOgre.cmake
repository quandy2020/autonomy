# FindOgre.cmake - Find Ogre3D (OGRE) rendering engine.
#
# Usage:
#   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#   find_package(Ogre REQUIRED)
#   target_link_libraries(your_target PRIVATE Ogre::OgreMain)
#
# Optional:
#   - OGRE_DIR: directory containing OGREConfig.cmake (config mode)
#   - OGRE_HOME: root of Ogre SDK (legacy)
#   - OGRE_STATIC: set to TRUE for static linking

include(FindPackageHandleStandardArgs)

# Prefer config mode if OGRE provides it (OGRE 2.x / modern installs)
set(_OGRE_DIR "")
if(OGRE_DIR)
  set(_OGRE_DIR "${OGRE_DIR}")
elseif(DEFINED ENV{OGRE_DIR})
  set(_OGRE_DIR "$ENV{OGRE_DIR}")
endif()
if(DEFINED ENV{OGRE_HOME})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{OGRE_HOME}")
  if(NOT _OGRE_DIR AND EXISTS "$ENV{OGRE_HOME}/lib/cmake/OGRE")
    set(_OGRE_DIR "$ENV{OGRE_HOME}/lib/cmake/OGRE")
  endif()
endif()

if(_OGRE_DIR OR CMAKE_PREFIX_PATH)
  find_package(OGRE QUIET CONFIG
    NAMES OGRE
    HINTS ${_OGRE_DIR}
    PATH_SUFFIXES lib/cmake/OGRE cmake OGRE
  )
endif()

if(OGRE_FOUND)
  set(Ogre_FOUND TRUE)
  if(TARGET Ogre::OgreMain)
    set(Ogre_OgreMain_FOUND TRUE)
  endif()
  if(TARGET Ogre::Bites)
    set(Ogre_Bites_FOUND TRUE)
  endif()
else()
  # Module mode: find include and library manually
  set(_OGRE_SEARCH_INC "")
  set(_OGRE_SEARCH_LIB "")
  if(DEFINED ENV{OGRE_HOME})
    list(APPEND _OGRE_SEARCH_INC "$ENV{OGRE_HOME}/include")
    list(APPEND _OGRE_SEARCH_INC "$ENV{OGRE_HOME}/include/OGRE")
    list(APPEND _OGRE_SEARCH_LIB "$ENV{OGRE_HOME}/lib" "$ENV{OGRE_HOME}/lib64")
  endif()
  list(APPEND _OGRE_SEARCH_INC "/usr/local/include" "/usr/local/include/OGRE")
  list(APPEND _OGRE_SEARCH_LIB "/usr/local/lib" "/usr/local/lib64" "/usr/lib" "/usr/lib64")

  find_path(OGRE_INCLUDE_DIR Ogre.h
    HINTS ${_OGRE_SEARCH_INC}
    PATH_SUFFIXES OGRE Ogre
  )
  find_library(OGRE_LIBRARY
    NAMES OgreMain OGRE
    HINTS ${_OGRE_SEARCH_LIB}
  )

  if(OGRE_INCLUDE_DIR AND OGRE_LIBRARY)
    set(Ogre_FOUND TRUE)
    set(Ogre_INCLUDE_DIRS "${OGRE_INCLUDE_DIR}")
    set(Ogre_LIBRARIES "${OGRE_LIBRARY}")
    if(NOT TARGET Ogre::OgreMain)
      add_library(Ogre::OgreMain UNKNOWN IMPORTED)
      set_target_properties(Ogre::OgreMain PROPERTIES
        IMPORTED_LOCATION "${OGRE_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OGRE_INCLUDE_DIR}"
      )
    endif()
  endif()

  find_package_handle_standard_args(Ogre
    REQUIRED_VARS Ogre_LIBRARIES Ogre_INCLUDE_DIRS
    FOUND_VAR Ogre_FOUND
  )
endif()

unset(_OGRE_DIR)
unset(_OGRE_SEARCH_INC)
unset(_OGRE_SEARCH_LIB)
