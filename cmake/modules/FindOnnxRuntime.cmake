# FindOnnxRuntime.cmake
# Finds ONNX Runtime (include + library) for C++ inference.
# Sets:
#   OnnxRuntime_FOUND
#   OnnxRuntime_INCLUDE_DIRS
#   OnnxRuntime_LIBRARIES

find_path(OnnxRuntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/local/include
    /usr/include
    ${OnnxRuntime_ROOT}/include
    $ENV{OnnxRuntime_ROOT}/include
)

find_library(OnnxRuntime_LIBRARY
  NAMES onnxruntime
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    ${OnnxRuntime_ROOT}/lib
    $ENV{OnnxRuntime_ROOT}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OnnxRuntime
  DEFAULT_MSG
  OnnxRuntime_LIBRARY
  OnnxRuntime_INCLUDE_DIR
)

if(OnnxRuntime_FOUND)
  set(OnnxRuntime_INCLUDE_DIRS ${OnnxRuntime_INCLUDE_DIR})
  set(OnnxRuntime_LIBRARIES ${OnnxRuntime_LIBRARY})
endif()

mark_as_advanced(OnnxRuntime_INCLUDE_DIR OnnxRuntime_LIBRARY)
