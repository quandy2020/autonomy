#=============================================================================
# FindONNXRuntime.cmake — ONNX Runtime for bolza mpcnet.
#
# Imported target: onnxruntime::onnxruntime
# Variables: ONNXRuntime_FOUND, ONNXRuntime_INCLUDE_DIRS, ONNXRuntime_LIBRARIES
#=============================================================================
include(FindPackageHandleStandardArgs)

find_path(ONNXRuntime_INCLUDE_DIR
  NAMES onnxruntime/onnxruntime_cxx_api.h
  HINTS
    ENV ONNXRUNTIME_ROOT
    ENV ONNXRuntime_DIR
  PATH_SUFFIXES include
  PATHS /usr /usr/local /opt/homebrew)

find_library(ONNXRuntime_LIBRARY
  NAMES onnxruntime
  HINTS
    ENV ONNXRUNTIME_ROOT
    ENV ONNXRuntime_DIR
  PATH_SUFFIXES lib lib64
  PATHS /usr /usr/local /opt/homebrew
        /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)

find_package_handle_standard_args(ONNXRuntime DEFAULT_MSG
  ONNXRuntime_LIBRARY ONNXRuntime_INCLUDE_DIR)

if(ONNXRuntime_FOUND)
  set(ONNXRuntime_INCLUDE_DIRS ${ONNXRuntime_INCLUDE_DIR})
  set(ONNXRuntime_LIBRARIES ${ONNXRuntime_LIBRARY})
  if(NOT TARGET onnxruntime::onnxruntime)
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
      IMPORTED_LOCATION "${ONNXRuntime_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${ONNXRuntime_INCLUDE_DIR}")
  endif()
  message(STATUS "ONNX Runtime: ${ONNXRuntime_LIBRARY}")
endif()

mark_as_advanced(ONNXRuntime_INCLUDE_DIR ONNXRuntime_LIBRARY)
