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

# Lite protoc plugin: injects #include <memory> and ptr typedefs into
# protoc-generated .pb.h via OpenForInsert (no separate wrapper file).
find_package(Protobuf REQUIRED)

set(AUTOMSGS_PROTOC_PLUGIN_LITE automsgs_protoc_plugin_lite)
add_executable(${AUTOMSGS_PROTOC_PLUGIN_LITE}
  Generator.cc
  generator_main.cc)
target_link_libraries(${AUTOMSGS_PROTOC_PLUGIN_LITE} PRIVATE
  protobuf::libprotoc
  protobuf::libprotobuf)
target_include_directories(${AUTOMSGS_PROTOC_PLUGIN_LITE} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
if(UNIX)
  target_link_libraries(${AUTOMSGS_PROTOC_PLUGIN_LITE} PRIVATE pthread)
endif()

include(GNUInstallDirs)
install(TARGETS ${AUTOMSGS_PROTOC_PLUGIN_LITE} DESTINATION ${CMAKE_INSTALL_BINDIR})
