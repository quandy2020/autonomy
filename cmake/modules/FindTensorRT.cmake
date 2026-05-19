# FindTensorRT.cmake
# Finds NVIDIA TensorRT (nvinfer, nvonnxparser) and CUDA runtime.
# Sets:
#   TensorRT_FOUND
#   TensorRT_INCLUDE_DIRS
#   TensorRT_LIBRARIES

find_path(TensorRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS
    ${TensorRT_ROOT}/include
    $ENV{TensorRT_ROOT}/include
    /usr/include
    /usr/local/include
    /usr/include/x86_64-linux-gnu
)

find_library(TensorRT_NVINFER_LIBRARY
  NAMES nvinfer
  PATHS
    ${TensorRT_ROOT}/lib
    ${TensorRT_ROOT}/lib/x64
    $ENV{TensorRT_ROOT}/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

find_library(TensorRT_NVONNXPARSER_LIBRARY
  NAMES nvonnxparser
  PATHS
    ${TensorRT_ROOT}/lib
    ${TensorRT_ROOT}/lib/x64
    $ENV{TensorRT_ROOT}/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

find_library(TensorRT_CUDART_LIBRARY
  NAMES cudart
  PATHS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    /usr/local/cuda/lib64
    /usr/lib/x86_64-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TensorRT
  DEFAULT_MSG
  TensorRT_NVINFER_LIBRARY
  TensorRT_INCLUDE_DIR
)

if(TensorRT_FOUND)
  set(TensorRT_INCLUDE_DIRS ${TensorRT_INCLUDE_DIR})
  set(TensorRT_LIBRARIES
    ${TensorRT_NVINFER_LIBRARY}
    ${TensorRT_NVONNXPARSER_LIBRARY}
    ${TensorRT_CUDART_LIBRARY})
endif()

mark_as_advanced(
  TensorRT_INCLUDE_DIR
  TensorRT_NVINFER_LIBRARY
  TensorRT_NVONNXPARSER_LIBRARY
  TensorRT_CUDART_LIBRARY)
