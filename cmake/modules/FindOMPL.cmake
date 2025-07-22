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

# Locate the OMPL library
find_path(OMPL_INCLUDE_DIR
    NAMES ompl/base/SpaceInformation.h
    PATHS /usr/local/include /usr/include /usr/local/include/ompl-1.6 /usr/include/ompl-1.5
)

find_library(OMPL_LIBRARY
  NAMES ompl
  PATHS /usr/local/lib /usr/lib
)

# Set the results
if(OMPL_INCLUDE_DIR AND OMPL_LIBRARY)
  set(OMPL_FOUND TRUE)
  set(OMPL_INCLUDE_DIRS ${OMPL_INCLUDE_DIR})
  set(OMPL_LIBRARIES ${OMPL_LIBRARY})
else()
  set(OMPL_FOUND FALSE)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OMPL DEFAULT_MSG OMPL_INCLUDE_DIR OMPL_LIBRARY)

mark_as_advanced(OMPL_INCLUDE_DIR OMPL_LIBRARY)
