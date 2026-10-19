# FindHesaiLidarSDK.cmake — HesaiLidar_SDK_2.0 (install_hesai_lidar_sdk.sh layout)
#
# Prefer the Config package written by scripts/install_hesai_lidar_sdk.sh:
#   <prefix>/lib/cmake/HesaiLidarSDK/HesaiLidarSDKConfig.cmake
# Fallback: hesai_lidar_sdk.hpp + static libs under opt/hesai_lidar_sdk.
#
# Hints: -DHesaiLidarSDK_ROOT=...  or  $HESAI_LIDAR_SDK_DIR / $HESAI_SDK_DIR

cmake_policy(PUSH)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(TARGET HesaiLidarSDK::hesai_sdk)
  set(HesaiLidarSDK_FOUND TRUE)
  cmake_policy(POP)
  return()
endif()

set(_HesaiLidarSDK_hints
  ${HesaiLidarSDK_ROOT}
  $ENV{HESAI_LIDAR_SDK_DIR}
  $ENV{HESAI_SDK_DIR}
  /usr/local
  /usr
)

# Config package from install_hesai_lidar_sdk.sh (INTERFACE target + include list).
find_package(HesaiLidarSDK CONFIG QUIET
  HINTS ${_HesaiLidarSDK_hints}
  PATH_SUFFIXES lib/cmake/HesaiLidarSDK)

if(TARGET HesaiLidarSDK::hesai_sdk)
  set(HesaiLidarSDK_FOUND TRUE)
  unset(_HesaiLidarSDK_hints)
  cmake_policy(POP)
  return()
endif()

# MODULE fallback: headers + static archives under PREFIX/opt/hesai_lidar_sdk.
find_path(HesaiLidarSDK_INCLUDE_DIR
  NAMES hesai_lidar_sdk.hpp
  HINTS ${_HesaiLidarSDK_hints}
  PATH_SUFFIXES
    include
    driver
    opt/hesai_lidar_sdk/driver
    hesai_lidar_sdk/driver
)

set(_HesaiLidarSDK_lib_names
  log_lib
  platutils_lib
  source_lib
  ptcClient_lib
  serialClient_lib
)

set(HesaiLidarSDK_LIBRARIES "")
set(_HesaiLidarSDK_libs_ok TRUE)
foreach(_lib IN LISTS _HesaiLidarSDK_lib_names)
  find_library(HesaiLidarSDK_${_lib}_LIBRARY
    NAMES ${_lib} lib${_lib}.a
    HINTS ${_HesaiLidarSDK_hints}
    PATH_SUFFIXES lib lib64 opt/hesai_lidar_sdk/lib)
  if(NOT HesaiLidarSDK_${_lib}_LIBRARY)
    set(_HesaiLidarSDK_libs_ok FALSE)
  else()
    list(APPEND HesaiLidarSDK_LIBRARIES "${HesaiLidarSDK_${_lib}_LIBRARY}")
  endif()
endforeach()

unset(_HesaiLidarSDK_hints)
unset(_HesaiLidarSDK_lib_names)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(HesaiLidarSDK
  REQUIRED_VARS HesaiLidarSDK_INCLUDE_DIR _HesaiLidarSDK_libs_ok)

if(HesaiLidarSDK_FOUND AND NOT TARGET HesaiLidarSDK::hesai_sdk)
  find_package(Threads REQUIRED)
  # Include roots mirror install_hesai_lidar_sdk.sh (driver + libhesai tree).
  get_filename_component(_hesai_driver_dir "${HesaiLidarSDK_INCLUDE_DIR}" ABSOLUTE)
  get_filename_component(_hesai_root "${_hesai_driver_dir}/.." ABSOLUTE)
  set(_hesai_incs
    "${_hesai_root}"
    "${_hesai_root}/driver"
    "${_hesai_root}/generated"
    "${_hesai_root}/libhesai"
    "${_hesai_root}/libhesai/Lidar"
    "${_hesai_root}/libhesai/Common/include"
    "${_hesai_root}/libhesai/Logger/include"
    "${_hesai_root}/libhesai/Source/include"
    "${_hesai_root}/libhesai/PtcClient/include"
    "${_hesai_root}/libhesai/PtcParser"
    "${_hesai_root}/libhesai/PtcParser/include"
    "${_hesai_root}/libhesai/SerialClient/include"
    "${_hesai_root}/libhesai/UdpParser/include"
    "${_hesai_root}/libhesai/Container/include"
    "${_hesai_root}/libhesai/Container/src"
  )
  # Link order: dependents first (serial/ptc/source → platutils → log).
  list(REVERSE HesaiLidarSDK_LIBRARIES)
  add_library(HesaiLidarSDK::hesai_sdk INTERFACE IMPORTED)
  set_target_properties(HesaiLidarSDK::hesai_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_hesai_incs}"
    INTERFACE_LINK_LIBRARIES "${HesaiLidarSDK_LIBRARIES};Threads::Threads")
  unset(_hesai_driver_dir)
  unset(_hesai_root)
  unset(_hesai_incs)
endif()

mark_as_advanced(HesaiLidarSDK_INCLUDE_DIR)
cmake_policy(POP)
