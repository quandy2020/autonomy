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

# Locate the FastRTPS library
find_path(FastRTPS_INCLUDE_DIR
  NAMES fastrtps/Domain.h
  PATHS /opt/cyber/include
)

find_library(FastRTPS_LIBRARY
  NAMES fastrtps
  PATHS /opt/cyber/lib
)

# Set the results
if(FastRTPS_INCLUDE_DIR AND FastRTPS_LIBRARY)
  set(FastRTPS_FOUND TRUE)
  set(FastRTPS_INCLUDE_DIRS ${FastRTPS_INCLUDE_DIR})
  set(FastRTPS_LIBRARIES ${FastRTPS_LIBRARY})
else()
  set(FastRTPS_FOUND FALSE)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FastRTPS DEFAULT_MSG FastRTPS_INCLUDE_DIR FastRTPS_LIBRARY)

mark_as_advanced(FastRTPS_INCLUDE_DIR FastRTPS_LIBRARY)
