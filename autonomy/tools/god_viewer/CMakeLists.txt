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

# Find all Foxglove SDK source files
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${foxglove_SOURCE_DIR}/src/*.cpp"
    "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

# include 
include_directories(${foxglove_SOURCE_DIR}/include)

set(god_viewer_name autonomy.tools.god_viewer)
# autonomy.tools.god_viewer
add_executable(${god_viewer_name}
  god_viewer_main.cpp
  viewer_bridge.cpp
  channel/channel_path_handler.cpp
  channel/channel_map_handler.cpp
)

target_link_libraries(${god_viewer_name} PUBLIC ${PROJECT_NAME})
target_link_libraries(${god_viewer_name} PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.so) 
target_sources(${god_viewer_name} PRIVATE ${FOXGLOVE_SOURCES})

# install
install(TARGETS
  ${god_viewer_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)