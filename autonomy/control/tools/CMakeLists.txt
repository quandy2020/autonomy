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

google_binary(autonomy_controller_test
  SRCS
    controller_server_test.cpp
)

if(BUILD_TEST)
  add_test(
    NAME autonomy_controller_test_cli
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/controller_test_cli.sh
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
  set_tests_properties(autonomy_controller_test_cli PROPERTIES
    ENVIRONMENT
      "AUTONOMY_BUILD_DIR=${CMAKE_BINARY_DIR};AUTONOMY_CONFIG_DIR=${PROJECT_SOURCE_DIR}/config"
    DEPENDS autonomy_controller_test
  )
endif()
