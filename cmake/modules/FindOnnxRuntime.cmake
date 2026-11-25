# @file FindOnnxRuntime.cmake
# @brief Find-module for the ONNX Runtime C++ inference library.
#
# Prefers OnnxRuntime_ROOT / $ENV{OnnxRuntime_ROOT}, then the workspace
# `.deps` SDK (survives /tmp wipes), then /usr/local.
#
# @var OnnxRuntime_FOUND OnnxRuntime_INCLUDE_DIRS OnnxRuntime_LIBRARIES

set(_ort_hints)
if(DEFINED OnnxRuntime_ROOT AND NOT OnnxRuntime_ROOT STREQUAL "")
  list(APPEND _ort_hints "${OnnxRuntime_ROOT}")
endif()
if(DEFINED ENV{OnnxRuntime_ROOT} AND NOT "$ENV{OnnxRuntime_ROOT}" STREQUAL "")
  list(APPEND _ort_hints "$ENV{OnnxRuntime_ROOT}")
endif()

# This file lives at <repo>/src/autonomy/cmake/modules/.
get_filename_component(_ort_repo_root
  "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
list(APPEND _ort_hints
  "${_ort_repo_root}/.deps/onnxruntime-linux-x64-1.23.2"
  "${CMAKE_SOURCE_DIR}/.deps/onnxruntime-linux-x64-1.23.2"
  "${CMAKE_SOURCE_DIR}/../../.deps/onnxruntime-linux-x64-1.23.2"
  "/tmp/autonomy-onnxruntime-1.23.2-sdk"
  "${CMAKE_INSTALL_PREFIX}"
  /usr/local
  /usr
)

find_path(OnnxRuntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${_ort_hints}
  PATH_SUFFIXES include
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/local/include
    /usr/include
)

find_library(OnnxRuntime_LIBRARY
  NAMES onnxruntime
  HINTS ${_ort_hints}
  PATH_SUFFIXES lib lib64
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
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

unset(_ort_hints)
unset(_ort_repo_root)

mark_as_advanced(OnnxRuntime_INCLUDE_DIR OnnxRuntime_LIBRARY)
