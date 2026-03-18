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

# C++ 工具：autolink_channel（对应 Autolink tools channel 命令）
add_executable(autolink_channel main.cpp)
target_include_directories(autolink_channel PRIVATE BEFORE
  ${CMAKE_SOURCE_DIR}
)
target_link_libraries(autolink_channel PRIVATE autolink)

install(TARGETS autolink_channel
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
