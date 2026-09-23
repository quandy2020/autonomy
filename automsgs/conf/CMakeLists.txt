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

# msgs<major>.yaml. The installed copy points at the install prefix.
# The build-tree copy points at a per-config test directory, so it goes
# through file(GENERATE).
set(_yaml "msgs${PROJECT_VERSION_MAJOR}.yaml")

set(automsgs_library_path "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/ruby/automsgs")
configure_file(msgs.yaml.in "${CMAKE_CURRENT_BINARY_DIR}/${_yaml}" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${_yaml}"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automsgs")

set(automsgs_library_path "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/automsgs")
configure_file(msgs.yaml.in "${CMAKE_CURRENT_BINARY_DIR}/${_yaml}.configured" @ONLY)
file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>/${_yaml}"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/${_yaml}.configured")
