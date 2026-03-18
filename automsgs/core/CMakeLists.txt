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

#-----------------------------------------------------------------------------
# Protoc plugins (optional; proto/ can use Python-only generation instead)
#-----------------------------------------------------------------------------
add_subdirectory(generator)
add_subdirectory(generator_lite)

#-----------------------------------------------------------------------------
# Config header
#-----------------------------------------------------------------------------
set(AUTOMSGS_MSGS_CONFIG_HH ${CMAKE_CURRENT_BINARY_DIR}/include/automsgs/msgs/config.hh)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/config.hh.in
  ${AUTOMSGS_MSGS_CONFIG_HH}
  @ONLY)

#-----------------------------------------------------------------------------
# InstallationDirectories implementation (embed install prefix)
#-----------------------------------------------------------------------------
set(AUTOMSGS_INSTALL_PREFIX_CC ${CMAKE_CURRENT_BINARY_DIR}/InstallationDirectories.cc)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/InstallationDirectories.cc.in
  ${AUTOMSGS_INSTALL_PREFIX_CC}
  @ONLY)

#-----------------------------------------------------------------------------
# Core library (config, getInstallPrefix, Factory, MessageFactory, DynamicFactory)
#-----------------------------------------------------------------------------
find_package(Protobuf REQUIRED)
add_library(automsgs_core SHARED
  ${AUTOMSGS_INSTALL_PREFIX_CC}
  src/Factory.cc
  src/MessageFactory.cc
  src/DynamicFactory.cc
)
target_link_libraries(automsgs_core PUBLIC protobuf::libprotobuf)
target_include_directories(automsgs_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_compile_definitions(automsgs_core PRIVATE automsgs_core_EXPORTS)
set_target_properties(automsgs_core PROPERTIES
  SOVERSION 1
  VERSION ${PROJECT_VERSION}
  POSITION_INDEPENDENT_CODE ON)

#-----------------------------------------------------------------------------
# Install
#-----------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS automsgs_core
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/Export.hh
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/InstallationDirectories.hh
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/Factory.hh
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/MessageFactory.hh
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/DynamicFactory.hh
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/Utility.hh
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/automsgs/msgs)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/automsgs/msgs/detail/dynamic_message_cast.hh
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/automsgs/msgs/detail)
install(FILES ${AUTOMSGS_MSGS_CONFIG_HH}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/automsgs/msgs)

add_subdirectory(cmd)
