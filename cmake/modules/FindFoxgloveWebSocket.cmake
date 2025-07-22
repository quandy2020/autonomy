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

# Locate the Foxglove WebSocket library
find_path(FoxgloveWebSocket_INCLUDE_DIR
    NAMES foxglove/websocket/websocket_client.hpp
    PATHS /usr/local/include
)

find_library(FoxgloveWebSocket_LIBRARY
  NAMES libfoxglove_websocket
  PATHS /usr/local/lib /usr/lib
)

# Set the results
if(FoxgloveWebSocket_INCLUDE_DIR AND FoxgloveWebSocket_LIBRARY)
  set(FoxgloveWebSocket_FOUND TRUE)
  set(FoxgloveWebSocket_INCLUDE_DIRS ${FoxgloveWebSocket_INCLUDE_DIR})
  set(FoxgloveWebSocket_LIBRARIES ${FoxgloveWebSocket_LIBRARY})
else()
  set(FoxgloveWebSocket_FOUND FALSE)
endif()

# Provide the package configuration
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FoxgloveWebSocket DEFAULT_MSG FoxgloveWebSocket_INCLUDE_DIR FoxgloveWebSocket_LIBRARY)

mark_as_advanced(FoxgloveWebSocket_INCLUDE_DIR FoxgloveWebSocket_LIBRARY)
