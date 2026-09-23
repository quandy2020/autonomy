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

set(AUTOMSGS_MSGS_CONFIG_HH "${CMAKE_CURRENT_BINARY_DIR}/include/automsgs/msgs/config.hh")
configure_file(include/automsgs/msgs/config.hh.in "${AUTOMSGS_MSGS_CONFIG_HH}" @ONLY)

set(AUTOMSGS_INSTALL_PREFIX_CC "${CMAKE_CURRENT_BINARY_DIR}/InstallationDirectories.cc")
configure_file(src/InstallationDirectories.cc.in "${AUTOMSGS_INSTALL_PREFIX_CC}" @ONLY)

target_sources(automsgs PRIVATE
  "${AUTOMSGS_INSTALL_PREFIX_CC}"
  src/Factory.cc
  src/MessageFactory.cc
  src/DynamicFactory.cc)
target_include_directories(automsgs PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

install(DIRECTORY include/automsgs/msgs/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/automsgs/msgs"
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.hpp")
install(FILES "${AUTOMSGS_MSGS_CONFIG_HH}"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/automsgs/msgs")

if(AUTOMSGS_BUILD_TOOLS)
  add_subdirectory(generator)
  add_subdirectory(generator_lite)
  add_subdirectory(cmd)
endif()
