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

# dependencies
set(timer_component_dependencies 
    autolink
    fastdds
    fastcdr
    pthread
    ${Protobuf_LIBRARIES}
)

add_library(timer_component_example SHARED
  timer_component_example.cpp
)

add_dependencies(timer_component_example examples_protos)

target_include_directories(timer_component_example PRIVATE 
    ${CMAKE_CURRENT_BINARY_DIR}/../proto
    ${CMAKE_CURRENT_SOURCE_DIR}/../proto
)

target_link_libraries(timer_component_example
  ${timer_component_dependencies}
)

set_target_properties(timer_component_example PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/autolink/examples/timer_component_example/"
)

file(COPY timer.dag timer.launch
  DESTINATION ${CMAKE_BINARY_DIR}/share/examples/timer_component_example
)

install(FILES timer.dag timer.launch
  DESTINATION share/examples/timer_component_example
)

install(TARGETS timer_component_example
    LIBRARY DESTINATION lib
)

