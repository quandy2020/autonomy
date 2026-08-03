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

autonomy_require_protobuf()

# Protoc plugin: generates .pb_index and wrapper .am.h (UniquePtr/SharedPtr typedefs)
set(AUTOMSGS_PROTOC_PLUGIN automsgs_protoc_plugin)
add_executable(${AUTOMSGS_PROTOC_PLUGIN}
  Generator.cc
  generator_main.cc)
target_link_libraries(${AUTOMSGS_PROTOC_PLUGIN} PRIVATE
  protobuf::libprotoc
  protobuf::libprotobuf)
target_include_directories(${AUTOMSGS_PROTOC_PLUGIN} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/core/include  # for generated code that includes automsgs/msgs/Export.hh
)
if(UNIX)
  target_link_libraries(${AUTOMSGS_PROTOC_PLUGIN} PRIVATE pthread)
endif()

include(GNUInstallDirs)
install(TARGETS ${AUTOMSGS_PROTOC_PLUGIN} DESTINATION ${CMAKE_INSTALL_BINDIR})
