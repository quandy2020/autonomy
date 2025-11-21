# Copyright 2016 The Cartographer Authors
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

# Locate the Stage library
find_path(STAGE_INCLUDE_DIR
    NAMES stage.hh
    PATHS /usr/local/include/Stage-4.3
)

find_library(STAGE_LIBRARY
  NAMES stage
  PATHS /usr/local/lib /usr/lib
)

# Set the results
if(STAGE_INCLUDE_DIR AND STAGE_LIBRARY)
  set(STAGE_FOUND TRUE)
  set(STAGE_INCLUDE_DIRS ${STAGE_INCLUDE_DIR})
  set(STAGE_LIBRARIES ${STAGE_LIBRARY})
else()
  set(STAGE_FOUND FALSE)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(STAGE DEFAULT_MSG STAGE_INCLUDE_DIR STAGE_LIBRARY)

mark_as_advanced(STAGE_INCLUDE_DIR STAGE_LIBRARY)
