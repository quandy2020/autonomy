# FindGenisomL1w.cmake — locate GENISOM / zsibot ZSL-1W high-level SDK
#
# Official SDK (legacy layout still used for L1-W HighLevel API):
#   https://github.com/zsibot/genisom_l1_sdk_old  (include/zsl-1w/highlevel.h)
#   or extracted package with the same layout
#
# Expects:
#   ${prefix}/include/zsl-1w/highlevel.h
#   ${prefix}/lib/zsl-1w/<arch>/libmc_sdk_zsl_1w_<arch>.so
#
# Hint: -DGenisomL1w_ROOT=/path/to/sdk  or  GENISOM_L1W_SDK_ROOT / ZSIBOT_L1_SDK
#
# Sets: GenisomL1w_FOUND, imported target GenisomL1w::GenisomL1w

cmake_policy(PUSH)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(_GenisomL1w_ARCH "aarch64")
else()
  set(_GenisomL1w_ARCH "x86_64")
endif()

set(_GenisomL1w_hints
  ${GenisomL1w_ROOT}
  $ENV{GENISOM_L1W_SDK_ROOT}
  $ENV{ZSIBOT_L1_SDK}
  $ENV{GENISOM_L1_SDK}
  /usr/local
  /opt/genisom_l1_sdk
  /opt/zsibot/genisom_l1_sdk
)

find_path(GenisomL1w_INCLUDE_DIR
  NAMES zsl-1w/highlevel.h
  HINTS ${_GenisomL1w_hints}
  PATH_SUFFIXES include
)

find_library(GenisomL1w_LIBRARY
  NAMES
    mc_sdk_zsl_1w_${_GenisomL1w_ARCH}
    mc_sdk_zsl_1w
  HINTS ${_GenisomL1w_hints}
  PATH_SUFFIXES
    lib/zsl-1w/${_GenisomL1w_ARCH}
    lib/zsl-1w
    lib
    lib64
)

unset(_GenisomL1w_hints)
unset(_GenisomL1w_ARCH)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GenisomL1w
  REQUIRED_VARS GenisomL1w_LIBRARY GenisomL1w_INCLUDE_DIR)

if(GenisomL1w_FOUND)
  set(GenisomL1w_INCLUDE_DIRS ${GenisomL1w_INCLUDE_DIR})
  set(GenisomL1w_LIBRARIES ${GenisomL1w_LIBRARY})
  if(NOT TARGET GenisomL1w::GenisomL1w)
    add_library(GenisomL1w::GenisomL1w UNKNOWN IMPORTED)
    set_target_properties(GenisomL1w::GenisomL1w PROPERTIES
      IMPORTED_LOCATION "${GenisomL1w_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${GenisomL1w_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(GenisomL1w_INCLUDE_DIR GenisomL1w_LIBRARY)
cmake_policy(POP)
