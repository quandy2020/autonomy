# FindLivoxSDK.cmake — installed Livox-SDK (v1)
#
# Expects: livox_sdk.h + liblivox_sdk_static.a
# Hint: -DLivoxSDK_ROOT=/usr/local

cmake_policy(PUSH)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

set(_LivoxSDK_hints
  ${LivoxSDK_ROOT}
  $ENV{LIVOX_SDK_DIR}
  $ENV{LIVOX_SDK_PATH}
  /usr/local
  /usr
)

find_path(LivoxSDK_INCLUDE_DIR
  NAMES livox_sdk.h
  HINTS ${_LivoxSDK_hints}
  PATH_SUFFIXES include
)

find_library(LivoxSDK_LIBRARY
  NAMES livox_sdk_static livox_sdk
  HINTS ${_LivoxSDK_hints}
  PATH_SUFFIXES lib lib64
)

unset(_LivoxSDK_hints)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LivoxSDK
  REQUIRED_VARS LivoxSDK_LIBRARY LivoxSDK_INCLUDE_DIR)

if(LivoxSDK_FOUND AND NOT TARGET LivoxSDK::LivoxSDK)
  add_library(LivoxSDK::LivoxSDK UNKNOWN IMPORTED)
  set_target_properties(LivoxSDK::LivoxSDK PROPERTIES
    IMPORTED_LOCATION "${LivoxSDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LivoxSDK_INCLUDE_DIR}")
endif()

mark_as_advanced(LivoxSDK_INCLUDE_DIR LivoxSDK_LIBRARY)
cmake_policy(POP)
