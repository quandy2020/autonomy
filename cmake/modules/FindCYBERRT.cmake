# Copyright 2024 The OpenRobotic Beginner Authors
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

# Locate the CyberRT library
find_path(CYBERRT_INCLUDE_DIR
  NAMES cyber/cyber.h
  PATHS /opt/cyber/include
)

find_library(CYBERRT_LIBRARY
  NAMES cyber
  PATHS /opt/cyber/lib
)

# Set the results
if(CYBERRT_INCLUDE_DIR AND CYBERRT_LIBRARY)
  set(CYBERRT_FOUND TRUE)
  set(CYBERRT_INCLUDE_DIRS ${CYBERRT_INCLUDE_DIR})
  set(CYBERRT_LIBRARIES ${CYBERRT_LIBRARY})
else()
  set(CYBERRT_FOUND FALSE)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CyberRT DEFAULT_MSG CYBERRT_INCLUDE_DIR CYBERRT_LIBRARY)

mark_as_advanced(CYBERRT_INCLUDE_DIR CYBERRT_LIBRARY)
