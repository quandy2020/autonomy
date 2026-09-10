# FindLivoxSDK2.cmake — installed Livox-SDK2
#
# Expects: livox_lidar_api.h + liblivox_lidar_sdk_static.a (or shared)
# Hint: -DLivoxSDK2_ROOT=/usr/local

cmake_policy(PUSH)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(_LivoxSDK2_hints
  ${LivoxSDK2_ROOT}
  $ENV{LIVOX_SDK2_DIR}
  $ENV{LIVOX_SDK2_PATH}
  /usr/local
  /usr
)

find_path(LivoxSDK2_INCLUDE_DIR
  NAMES livox_lidar_api.h
  HINTS ${_LivoxSDK2_hints}
  PATH_SUFFIXES include
)

# Prefer shared: static .a from upstream is often built without -fPIC and
# cannot be linked into libautodriver.so.
find_library(LivoxSDK2_LIBRARY
  NAMES livox_lidar_sdk_shared livox_lidar_sdk livox_lidar_sdk_static
  HINTS ${_LivoxSDK2_hints}
  PATH_SUFFIXES lib lib64
)

unset(_LivoxSDK2_hints)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LivoxSDK2
  REQUIRED_VARS LivoxSDK2_LIBRARY LivoxSDK2_INCLUDE_DIR)

if(LivoxSDK2_FOUND AND NOT TARGET LivoxSDK2::LivoxSDK2)
  add_library(LivoxSDK2::LivoxSDK2 UNKNOWN IMPORTED)
  set_target_properties(LivoxSDK2::LivoxSDK2 PROPERTIES
    IMPORTED_LOCATION "${LivoxSDK2_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LivoxSDK2_INCLUDE_DIR}")
endif()

mark_as_advanced(LivoxSDK2_INCLUDE_DIR LivoxSDK2_LIBRARY)
cmake_policy(POP)
