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

# autonomy/orbisview — CivetWeb HTTP/WS HMI (companion; BUILD_ORBISVIEW)

include(autonomy_module)
autonomy_module(orbisview)

add_subdirectory(thirdparty/civetweb)

file(GLOB_RECURSE _ORBISVIEW_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/backend/*.cpp")
list(FILTER _ORBISVIEW_SRCS EXCLUDE REGEX "_test\\.cpp$")
list(FILTER _ORBISVIEW_SRCS EXCLUDE REGEX "/plugins/examples/")

autonomy_library(
  NO_CORE
  DEPENDENCIES autolink automsgs orbisview_civetweb glog::glog gflags::gflags
  SRCS ${_ORBISVIEW_SRCS})
unset(_ORBISVIEW_SRCS)

# Companion: not a domain module — keep out of AutonomyExport
# (PUBLIC orbisview_civetweb is not in that export set).
get_property(_t GLOBAL PROPERTY AUTONOMY_MODULE_TARGETS)
list(REMOVE_ITEM _t autonomy_orbisview)
set_property(GLOBAL PROPERTY AUTONOMY_MODULE_TARGETS "${_t}")
unset(_t)

# Source still gates Autolink/automsgs behind these macros.
target_compile_definitions(autonomy_orbisview PUBLIC
  ORBISVIEW_WITH_AUTOLINK=1 ORBISVIEW_WITH_AUTOMSGS=1)

autonomy_binary(autonomy.orbisview
  SRCS main.cpp
  DEPENDENCIES autonomy_orbisview)

add_subdirectory(frontend)

if(BUILD_TEST)
  autonomy_test(orbisview.throttle_queue_test
    backend/common/streaming/throttle_queue_test.cpp autonomy_orbisview)
  autonomy_test(orbisview.stream_envelope_test
    backend/common/stream_envelope_test.cpp autonomy_orbisview)
  autonomy_test(orbisview.envelope_recorder_test
    backend/record_player/envelope_recorder_test.cpp autonomy_orbisview)
  autonomy_test(orbisview.plugin_host_test
    backend/common/plugins/plugin_host_test.cpp autonomy_orbisview)
  autonomy_test(orbisview.automsgs_converter_test
    backend/adapters/automsgs/automsgs_converter_test.cpp autonomy_orbisview)
endif()

# Runtime assets → share/autonomy/orbisview/ (conf / launch / script; www via frontend/)
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/orbisview/conf
  FILES_MATCHING PATTERN "*.conf" PATTERN "*.json" PATTERN "README.md")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/orbisview/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch")
install(
  DIRECTORY script/
  DESTINATION share/autonomy/orbisview/script
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.sh")
