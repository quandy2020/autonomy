# Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(bolza_initialize_project)
  list(PREPEND CMAKE_MODULE_PATH "${BOLZA_ROOT_DIR}/cmake/modules")
endmacro()

function(bolza_test NAME ARG_SRC)
  add_executable(${NAME} ${ARG_SRC})
  target_link_libraries(${NAME} PUBLIC bolza)
  if(TARGET GTest::gtest_main)
    target_link_libraries(${NAME} PUBLIC GTest::gtest_main)
  elseif(GMOCK_LIBRARIES)
    target_include_directories(${NAME} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
    target_link_libraries(${NAME} PUBLIC ${GMOCK_LIBRARIES})
  else()
    message(FATAL_ERROR "bolza_test(${NAME}): GTest/GMock not found")
  endif()
  add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

macro(bolza_enable_testing)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    find_package(GMock QUIET)
  endif()
  if(NOT GTest_FOUND AND NOT GMock_FOUND)
    message(FATAL_ERROR "bolza: BOLZA_BUILD_TESTS=ON but GTest/GMock not found")
  endif()
endmacro()
