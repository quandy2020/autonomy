# FindQt.cmake - Find Qt5 or Qt6 (prefer Qt6 if available).
#
# Usage:
#   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#   find_package(Qt REQUIRED COMPONENTS Widgets Core)
#   target_link_libraries(your_target PRIVATE Qt::Widgets Qt::Core)
#
# Sets:
#   Qt_FOUND, Qt_VERSION_MAJOR (5 or 6)
#   Targets: Qt::Core, Qt::Widgets (and others per COMPONENTS)
#
# Optional:
#   - Qt_PREFER_QT6: if set, try Qt6 first (default ON)
#   - Qt_PREFER_QT5: if set, try Qt5 first and skip Qt6

include(FindPackageHandleStandardArgs)

set(Qt_FOUND FALSE)
set(Qt_VERSION_MAJOR 0)

if(Qt_PREFER_QT5)
  set(_TRY_QT6 FALSE)
  set(_TRY_QT5 TRUE)
else()
  set(_TRY_QT6 TRUE)
  set(_TRY_QT5 TRUE)
endif()

# Try Qt6
if(_TRY_QT6)
  find_package(Qt6 QUIET COMPONENTS Widgets Core)
  if(Qt6_FOUND)
    set(Qt_FOUND TRUE)
    set(Qt_VERSION_MAJOR 6)
    set(Qt_VERSION ${Qt6_VERSION})
    if(NOT TARGET Qt::Core)
      add_library(Qt::Core ALIAS Qt6::Core)
    endif()
    if(NOT TARGET Qt::Widgets)
      add_library(Qt::Widgets ALIAS Qt6::Widgets)
    endif()
  endif()
endif()

# Fallback to Qt5
if(NOT Qt_FOUND AND _TRY_QT5)
  find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
  if(Qt5_FOUND)
    set(Qt_FOUND TRUE)
    set(Qt_VERSION_MAJOR 5)
    set(Qt_VERSION ${Qt5_VERSION})
    if(NOT TARGET Qt::Core)
      add_library(Qt::Core ALIAS Qt5::Core)
    endif()
    if(NOT TARGET Qt::Widgets)
      add_library(Qt::Widgets ALIAS Qt5::Widgets)
    endif()
  endif()
endif()

find_package_handle_standard_args(Qt
  REQUIRED_VARS Qt_FOUND Qt_VERSION_MAJOR
  VERSION_VAR Qt_VERSION
  FOUND_VAR Qt_FOUND
)

unset(_TRY_QT6)
unset(_TRY_QT5)
