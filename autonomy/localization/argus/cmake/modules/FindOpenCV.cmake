# Copyright 2026 The Openbot Authors
#
# find_package(OpenCV REQUIRED [COMPONENTS ...])
#   → OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBRARIES, OpenCV_VERSION
#   → OpenCV::OpenCV (INTERFACE umbrella) when component IMPORTED targets exist
#
# Components used by Argus: core, imgproc, imgcodecs, and calib3d (OpenCV≤4)
# or calib (OpenCV 5+). geometry is linked when present (undistortPoints).

include(FindPackageHandleStandardArgs)

if(OpenCV_FOUND AND (TARGET OpenCV::OpenCV OR TARGET opencv_core OR
                     DEFINED OpenCV_LIBRARIES))
  find_package_handle_standard_args(OpenCV DEFAULT_MSG OpenCV_LIBRARIES)
  return()
endif()

# Homebrew / local Config hints (OpenCV 4 and 5 layouts).
foreach(_opencv_prefix IN ITEMS
    "/opt/homebrew"
    "/usr/local"
    "$ENV{HOME}/.local")
  foreach(_opencv_cfg IN ITEMS
      "${_opencv_prefix}/lib/cmake/opencv4/OpenCVConfig.cmake"
      "${_opencv_prefix}/lib/cmake/opencv5/OpenCVConfig.cmake"
      "${_opencv_prefix}/share/opencv4/OpenCVConfig.cmake"
      "${_opencv_prefix}/share/OpenCV/OpenCVConfig.cmake")
    if(EXISTS "${_opencv_cfg}")
      get_filename_component(OpenCV_DIR "${_opencv_cfg}" DIRECTORY CACHE)
      break()
    endif()
  endforeach()
  if(OpenCV_DIR)
    break()
  endif()
endforeach()

set(_ARGUS_OPENCV_COMPONENTS core imgproc imgcodecs)
if(OpenCV_FIND_COMPONENTS)
  set(_ARGUS_OPENCV_COMPONENTS ${OpenCV_FIND_COMPONENTS})
endif()

# First try requested components; then add calibration helper.
find_package(OpenCV QUIET NO_MODULE COMPONENTS ${_ARGUS_OPENCV_COMPONENTS})
if(NOT OpenCV_FOUND)
  find_package(OpenCV QUIET NO_MODULE)
endif()

# OpenCV 5: calib3d → calib; undistortPoints may live in geometry.
if(OpenCV_FOUND)
  if(NOT OpenCV_LIBRARIES AND OpenCV_LIBS)
    set(OpenCV_LIBRARIES ${OpenCV_LIBS})
  endif()
  if(TARGET opencv_calib3d)
    list(APPEND OpenCV_LIBRARIES opencv_calib3d)
  elseif(TARGET opencv_calib)
    list(APPEND OpenCV_LIBRARIES opencv_calib)
  endif()
  if(TARGET opencv_geometry)
    list(APPEND OpenCV_LIBRARIES opencv_geometry)
  endif()
  if(OpenCV_LIBRARIES)
    list(REMOVE_DUPLICATES OpenCV_LIBRARIES)
  endif()
endif()

if(NOT OpenCV_FOUND)
  find_path(OpenCV_INCLUDE_DIR
    NAMES opencv2/core.hpp
    HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH} $ENV{OpenCV_DIR}
    PATHS
      /opt/homebrew/include/opencv5
      /opt/homebrew/include/opencv4
      /usr/local/include/opencv4
      /usr/include/opencv4
      /usr/include
    PATH_SUFFIXES include opencv4 opencv5
  )

  set(_opencv_lib_names opencv_core opencv_imgproc opencv_imgcodecs)
  set(OpenCV_LIBRARIES "")
  set(_opencv_lib_ok TRUE)
  foreach(_lib ${_opencv_lib_names})
    find_library(_opencv_${_lib}_path
      NAMES ${_lib}
      HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
      PATHS /opt/homebrew /usr/local /usr
      PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu
    )
    if(_opencv_${_lib}_path)
      list(APPEND OpenCV_LIBRARIES ${_opencv_${_lib}_path})
    else()
      set(_opencv_lib_ok FALSE)
    endif()
  endforeach()

  # Optional calibration libs (name differs across major versions).
  foreach(_lib IN ITEMS opencv_calib3d opencv_calib opencv_geometry)
    find_library(_opencv_${_lib}_path
      NAMES ${_lib}
      HINTS ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH}
      PATHS /opt/homebrew /usr/local /usr
      PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu
    )
    if(_opencv_${_lib}_path)
      list(APPEND OpenCV_LIBRARIES ${_opencv_${_lib}_path})
    endif()
  endforeach()

  if(OpenCV_INCLUDE_DIR AND _opencv_lib_ok)
    set(OpenCV_FOUND TRUE)
    set(OpenCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIR}")
    if(NOT DEFINED OpenCV_VERSION)
      set(OpenCV_VERSION "0.0.0")
    endif()
  endif()
endif()

if(OpenCV_FOUND AND NOT TARGET OpenCV::OpenCV)
  add_library(OpenCV::OpenCV INTERFACE IMPORTED)
  if(OpenCV_INCLUDE_DIRS)
    set_target_properties(OpenCV::OpenCV PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}")
  elseif(OpenCV_INCLUDE_DIR)
    set_target_properties(OpenCV::OpenCV PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIR}")
  endif()
  if(OpenCV_LIBRARIES)
    set_target_properties(OpenCV::OpenCV PROPERTIES
      INTERFACE_LINK_LIBRARIES "${OpenCV_LIBRARIES}")
  endif()
endif()

find_package_handle_standard_args(OpenCV
  REQUIRED_VARS OpenCV_LIBRARIES
  VERSION_VAR OpenCV_VERSION)

mark_as_advanced(OpenCV_DIR OpenCV_INCLUDE_DIR)
unset(_ARGUS_OPENCV_COMPONENTS)
unset(_opencv_prefix)
unset(_opencv_cfg)
unset(_opencv_lib_names)
unset(_opencv_lib_ok)
unset(_lib)
