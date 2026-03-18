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

# Designation for this component (used in output filename: msgs<MAJOR>.yaml)
set(AUTOMSGS_DESIGNATION "msgs")

# Path to Ruby cmd scripts for internal testing (build tree)
set(automsgs_library_path "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/automsgs")

# Generate configuration file for internal testing.
# Output: msgs1.yaml (version major in name)
configure_file(
  "${AUTOMSGS_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${AUTOMSGS_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml.configured"
  @ONLY)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>/${AUTOMSGS_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/${AUTOMSGS_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml.configured")

# Path for installed version
set(automsgs_library_path "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/ruby/automsgs")

# Generate the configuration file that is installed.
configure_file(
  "${AUTOMSGS_DESIGNATION}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${AUTOMSGS_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml"
  @ONLY)

# Install the yaml configuration files
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${AUTOMSGS_DESIGNATION}${PROJECT_VERSION_MAJOR}.yaml"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/automsgs)
