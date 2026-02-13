# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

# Locate the GLOG library
find_path(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  PATHS /usr/local/include
)

find_library(GLOG_LIBRARY
  NAMES glog
  PATHS /usr/local/lib
        /usr/local/lib64
)

# Set the results
if(GLOG_INCLUDE_DIR AND GLOG_LIBRARY)
  set(GLOG_FOUND TRUE)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
else()
  set(GLOG_FOUND FALSE)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog DEFAULT_MSG GLOG_INCLUDE_DIR GLOG_LIBRARY)

mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)


