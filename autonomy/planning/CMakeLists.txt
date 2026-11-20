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

# -----------------------------------------------------------------------------
# autonomy/planning — global / local planners
#
# Builds
#   autonomy_planning / autonomy::autonomy_planning
#   autonomy.planning
#
# Toggle
#   AUTONOMY_BUILD_PLANNING
# -----------------------------------------------------------------------------

# --- sources -----------------------------------------------------------------
file(GLOB_RECURSE AUTONOMY_PLANNING_SRCS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER AUTONOMY_PLANNING_SRCS EXCLUDE REGEX "main\\.cpp$")
list(FILTER AUTONOMY_PLANNING_SRCS EXCLUDE REGEX "_test\\.cpp$")

# Fields2Cover is an optional coverage domain, independent of Vista exploration.
# Keep its sources out of the core library until the application enables it.
option(AUTONOMY_BUILD_FIELDS2COVER "Build the optional Fields2Cover coverage planner" OFF)
set(AUTONOMY_PLANNING_OPTIONAL_DEPENDENCIES)
if(NOT AUTONOMY_BUILD_FIELDS2COVER)
  list(FILTER AUTONOMY_PLANNING_SRCS EXCLUDE REGEX "/fields2cover/")
endif()

# --- library -----------------------------------------------------------------
autocmake_library(autonomy_planning
  SHARED
  NO_EXPORT
  SOURCES ${AUTONOMY_PLANNING_SRCS}
  INCLUDES
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/planner
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/fields2cover
  DEPENDENCIES
    autonomy_common
    autonomy_proto
    autonomy_transform
    autonomy_map
    ${AUTONOMY_PLANNING_OPTIONAL_DEPENDENCIES})
add_library(autonomy::planning ALIAS autonomy_planning)

if(AUTONOMY_BUILD_FIELDS2COVER)
  find_package(Boost REQUIRED)
  target_include_directories(autonomy_planning PRIVATE ${Boost_INCLUDE_DIRS})
  target_include_directories(autonomy_planning PRIVATE
    ${PROJECT_SOURCE_DIR}/autolink/thirdparty/pugixml/src)
  target_compile_definitions(autonomy_planning PRIVATE PUGIXML_HEADER_ONLY)
  option(AUTONOMY_BUILD_FIELDS2COVER_TESTS "Build Fields2Cover regression tests" ${BUILD_TEST})
  if(AUTONOMY_BUILD_FIELDS2COVER_TESTS)
    enable_testing()
    add_executable(planning.fields2cover_test planner/fields2cover/fields2cover_test.cpp)
    target_link_libraries(planning.fields2cover_test PRIVATE autonomy_planning)
    add_test(NAME planning.fields2cover_test COMMAND planning.fields2cover_test)
    set_tests_properties(planning.fields2cover_test PROPERTIES TIMEOUT 60)
  endif()
endif()

# --- binary ------------------------------------------------------------------
autocmake_binary(autonomy.planning
  NO_EXPORT
  SOURCES planning_main.cpp
  DEPENDENCIES
    autonomy
    glog::glog
    autonomy::cli11
)

# --- install -----------------------------------------------------------------
autocmake_install_share(planning
  CONF
  DAG_PATTERN "*.dag"
  LAUNCH_PATTERN "*.launch")
