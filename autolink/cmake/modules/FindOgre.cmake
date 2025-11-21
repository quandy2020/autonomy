# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# FindOgre.cmake

# Locate Ogre library
# This module defines the following variables:
#  OGRE_FOUND - True if the library was found
#  OGRE_INCLUDE_DIRS - Include directories for Ogre
#  OGRE_LIBRARIES - Libraries to link against

find_path(OGRE_INCLUDE_DIR
  NAMES OGRE/Ogre.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

find_library(OGRE_LIBRARY
  NAMES OgreMain
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_PREFIX_PATH}/lib
)

find_library(OGRE_OVERLAY_LIBRARY
  NAMES OgreOverlay
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_PREFIX_PATH}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ogre DEFAULT_MSG
  OGRE_INCLUDE_DIR OGRE_LIBRARY)

if (OGRE_FOUND)
  set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIR})
  set(OGRE_LIBRARIES ${OGRE_LIBRARY})
  if (OGRE_OVERLAY_LIBRARY)
    set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${OGRE_OVERLAY_LIBRARY})
  endif()
endif()
