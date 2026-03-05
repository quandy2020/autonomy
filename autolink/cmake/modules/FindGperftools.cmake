# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Locate the Gperftools library
# Gperftools includes:
#   - libprofiler: CPU profiling
#   - tcmalloc/tcmalloc_and_profiler: Heap profiling and memory allocation

# Find profiler library
find_path(GPERFTOOLS_INCLUDE_DIR
  NAMES gperftools/profiler.h
  PATHS /usr/local/include
        /usr/include
  PATH_SUFFIXES gperftools
)

# Try to find tcmalloc_and_profiler first (contains both profiler and heap profiler)
find_library(GPERFTOOLS_TCMALLOC_AND_PROFILER_LIBRARY
  NAMES tcmalloc_and_profiler
  PATHS /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu
)

# Find profiler library separately
find_library(GPERFTOOLS_PROFILER_LIBRARY
  NAMES profiler
  PATHS /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu
)

# Find tcmalloc library (for HeapProfiler functions)
find_library(GPERFTOOLS_TCMALLOC_LIBRARY
  NAMES tcmalloc tcmalloc_minimal
  PATHS /usr/local/lib
        /usr/local/lib64
        /usr/lib
        /usr/lib64
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu
)

# Set the results
if(GPERFTOOLS_TCMALLOC_AND_PROFILER_LIBRARY)
  # If we found tcmalloc_and_profiler, use it (contains everything)
  set(GPERFTOOLS_FOUND TRUE)
  set(GPERFTOOLS_LIBRARIES ${GPERFTOOLS_TCMALLOC_AND_PROFILER_LIBRARY})
  if(NOT GPERFTOOLS_INCLUDE_DIR)
    # Try pkg-config as fallback for include directory
    pkg_check_modules(GPERFTOOLS_PC QUIET libprofiler)
    if(GPERFTOOLS_PC_FOUND AND GPERFTOOLS_PC_INCLUDE_DIRS)
      set(GPERFTOOLS_INCLUDE_DIRS ${GPERFTOOLS_PC_INCLUDE_DIRS})
    else()
      set(GPERFTOOLS_INCLUDE_DIRS /usr/local/include /usr/include)
    endif()
  else()
    set(GPERFTOOLS_INCLUDE_DIRS ${GPERFTOOLS_INCLUDE_DIR})
  endif()
elseif(GPERFTOOLS_PROFILER_LIBRARY AND GPERFTOOLS_TCMALLOC_LIBRARY)
  # If we found both separately, use both
  set(GPERFTOOLS_FOUND TRUE)
  set(GPERFTOOLS_LIBRARIES ${GPERFTOOLS_PROFILER_LIBRARY} ${GPERFTOOLS_TCMALLOC_LIBRARY})
  if(NOT GPERFTOOLS_INCLUDE_DIR)
    pkg_check_modules(GPERFTOOLS_PC QUIET libprofiler)
    if(GPERFTOOLS_PC_FOUND AND GPERFTOOLS_PC_INCLUDE_DIRS)
      set(GPERFTOOLS_INCLUDE_DIRS ${GPERFTOOLS_PC_INCLUDE_DIRS})
    else()
      set(GPERFTOOLS_INCLUDE_DIRS /usr/local/include /usr/include)
    endif()
  else()
    set(GPERFTOOLS_INCLUDE_DIRS ${GPERFTOOLS_INCLUDE_DIR})
  endif()
elseif(GPERFTOOLS_PROFILER_LIBRARY)
  # If we only found profiler, it's still useful for CPU profiling
  set(GPERFTOOLS_FOUND TRUE)
  set(GPERFTOOLS_LIBRARIES ${GPERFTOOLS_PROFILER_LIBRARY})
  if(NOT GPERFTOOLS_INCLUDE_DIR)
    pkg_check_modules(GPERFTOOLS_PC QUIET libprofiler)
    if(GPERFTOOLS_PC_FOUND AND GPERFTOOLS_PC_INCLUDE_DIRS)
      set(GPERFTOOLS_INCLUDE_DIRS ${GPERFTOOLS_PC_INCLUDE_DIRS})
    else()
      set(GPERFTOOLS_INCLUDE_DIRS /usr/local/include /usr/include)
    endif()
  else()
    set(GPERFTOOLS_INCLUDE_DIRS ${GPERFTOOLS_INCLUDE_DIR})
  endif()
else()
  set(GPERFTOOLS_FOUND FALSE)
  set(GPERFTOOLS_LIBRARIES)
  set(GPERFTOOLS_INCLUDE_DIRS)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gperftools
  DEFAULT_MSG
  GPERFTOOLS_LIBRARIES
  GPERFTOOLS_INCLUDE_DIRS
)

# Set standard variables for compatibility (mixed case matching package name)
if(Gperftools_FOUND)
  set(Gperftools_LIBRARIES ${GPERFTOOLS_LIBRARIES})
  set(Gperftools_INCLUDE_DIRS ${GPERFTOOLS_INCLUDE_DIRS})
endif()

mark_as_advanced(
  GPERFTOOLS_INCLUDE_DIR
  GPERFTOOLS_INCLUDE_DIRS
  GPERFTOOLS_PROFILER_LIBRARY
  GPERFTOOLS_TCMALLOC_LIBRARY
  GPERFTOOLS_TCMALLOC_AND_PROFILER_LIBRARY
  GPERFTOOLS_LIBRARIES
)

