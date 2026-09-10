# FindSherpaOnnx.cmake
# Finds sherpa-onnx C API (include + library) for offline/online ASR.
# Sets:
#   SherpaOnnx_FOUND
#   SherpaOnnx_INCLUDE_DIRS
#   SherpaOnnx_LIBRARIES

find_path(SherpaOnnx_INCLUDE_DIR
  NAMES sherpa-onnx/c-api/c-api.h
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/local/include
    /usr/include
    ${SherpaOnnx_ROOT}/include
    $ENV{SherpaOnnx_ROOT}/include
)

find_library(SherpaOnnx_LIBRARY
  NAMES sherpa-onnx-c-api
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    ${SherpaOnnx_ROOT}/lib
    $ENV{SherpaOnnx_ROOT}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SherpaOnnx
  DEFAULT_MSG
  SherpaOnnx_LIBRARY
  SherpaOnnx_INCLUDE_DIR
)

if(SherpaOnnx_FOUND)
  set(SherpaOnnx_INCLUDE_DIRS ${SherpaOnnx_INCLUDE_DIR})
  set(SherpaOnnx_LIBRARIES ${SherpaOnnx_LIBRARY})
endif()

mark_as_advanced(SherpaOnnx_INCLUDE_DIR SherpaOnnx_LIBRARY)
