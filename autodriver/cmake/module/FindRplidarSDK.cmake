# FindRplidarSDK.cmake — locate an installed Slamtec rplidar_sdk
#
# Install with: autodriver/scripts/install_rplidar_sdk.sh
# Expects:
#   ${prefix}/include/sl_lidar.h
#   ${prefix}/lib/libsl_lidar_sdk.a  (or .so)
#
# Hint: -DRplidarSDK_ROOT=/usr/local  or  RPLIDAR_SDK_DIR / RPLIDAR_SDK_PATH
#
# Sets: RplidarSDK_FOUND, RplidarSDK_INCLUDE_DIRS, RplidarSDK_LIBRARIES
# Imported target: RplidarSDK::RplidarSDK

cmake_policy(PUSH)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(_RplidarSDK_hints
  ${RplidarSDK_ROOT}
  $ENV{RPLIDAR_SDK_DIR}
  $ENV{RPLIDAR_SDK_PATH}
  /usr/local
  /usr
)

find_path(RplidarSDK_INCLUDE_DIR
  NAMES sl_lidar.h
  HINTS ${_RplidarSDK_hints}
  PATH_SUFFIXES include
)

find_library(RplidarSDK_LIBRARY
  NAMES sl_lidar_sdk
  HINTS ${_RplidarSDK_hints}
  PATH_SUFFIXES lib lib64
)

unset(_RplidarSDK_hints)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RplidarSDK
  REQUIRED_VARS RplidarSDK_LIBRARY RplidarSDK_INCLUDE_DIR)

if(RplidarSDK_FOUND)
  set(RplidarSDK_INCLUDE_DIRS ${RplidarSDK_INCLUDE_DIR})
  set(RplidarSDK_LIBRARIES ${RplidarSDK_LIBRARY})
  if(NOT TARGET RplidarSDK::RplidarSDK)
    add_library(RplidarSDK::RplidarSDK UNKNOWN IMPORTED)
    set_target_properties(RplidarSDK::RplidarSDK PROPERTIES
      IMPORTED_LOCATION "${RplidarSDK_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${RplidarSDK_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(RplidarSDK_INCLUDE_DIR RplidarSDK_LIBRARY)
cmake_policy(POP)
