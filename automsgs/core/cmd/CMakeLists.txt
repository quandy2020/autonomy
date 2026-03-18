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

set(msgs_executable_output_name "automsgs-msgs")
set(msgs_executable "automsgs-msgs-exe")
add_executable(${msgs_executable} msgs_main.cc)
set_target_properties(${msgs_executable} PROPERTIES OUTPUT_NAME ${msgs_executable_output_name})
target_link_libraries(${msgs_executable} PRIVATE automsgs_core)

include(GNUInstallDirs)
set(EXE_INSTALL_DIR ${CMAKE_INSTALL_BINDIR})
install(TARGETS ${msgs_executable} DESTINATION ${EXE_INSTALL_DIR})

# Ruby wrapper: path to exe relative to install dir of this script (lib/ruby/automsgs)
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmdautomsgs${PROJECT_VERSION_MAJOR}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")
set(executable_location "../../${EXE_INSTALL_DIR}/${msgs_executable_output_name}")
configure_file(
  "cmdautomsgs.rb.in"
  "${cmd_script_configured}"
  @ONLY)
file(GENERATE OUTPUT "${cmd_script_generated}" INPUT "${cmd_script_configured}")
install(FILES ${cmd_script_generated} DESTINATION ${CMAKE_INSTALL_LIBDIR}/ruby/automsgs)

# Bash completion
configure_file(
  "msgs.bash_completion.sh"
  "${CMAKE_CURRENT_BINARY_DIR}/msgs${PROJECT_VERSION_MAJOR}.bash_completion.sh"
  @ONLY)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/msgs${PROJECT_VERSION_MAJOR}.bash_completion.sh
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs/completion.d)
