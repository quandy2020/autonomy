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
add_library(automsgs_core_obj OBJECT
  ${AUTOMSGS_INSTALL_PREFIX_CC}
  src/Factory.cc
  src/MessageFactory.cc
  src/DynamicFactory.cc
)
target_include_directories(automsgs_core_obj
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_compile_definitions(automsgs_core_obj PRIVATE automsgs_core_EXPORTS)
set_target_properties(automsgs_core_obj PROPERTIES POSITION_INDEPENDENT_CODE ON)

#-----------------------------------------------------------------------------
# Install
#-----------------------------------------------------------------------------
include(GNUInstallDirs)
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

#-----------------------------------------------------------------------------
# Merge: build a single libautomsgs.so from core+proto objects
#-----------------------------------------------------------------------------
if(TARGET automsgs_proto_obj)
  add_library(automsgs SHARED
    $<TARGET_OBJECTS:automsgs_core_obj>
    $<TARGET_OBJECTS:automsgs_proto_obj>
  )

  target_link_libraries(automsgs PUBLIC protobuf::libprotobuf)
  target_include_directories(automsgs
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
  )

  # Expose generated proto headers include dirs for consumers in the build tree.
  target_include_directories(automsgs
    PUBLIC
      $<BUILD_INTERFACE:${AUTOMSGS_PROTO_INCLUDE_DIR}>
      $<BUILD_INTERFACE:${AUTOMSGS_PROTO_GEN_DIR}>
  )

  set_target_properties(automsgs PROPERTIES
    VERSION ${PROJECT_VERSION}
    POSITION_INDEPENDENT_CODE ON
  )

  include(GNUInstallDirs)
  install(TARGETS automsgs
    EXPORT automsgsTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  # Keep backward-compatible target names in this build tree.
  add_library(automsgs_core ALIAS automsgs)
  add_library(automsgs_proto ALIAS automsgs)
endif()

add_subdirectory(cmd)
