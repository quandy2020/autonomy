# Copyright 2024 The Openbot Authors (duyongquan)
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

# TestPlugins1 
add_library(${PROJECT_NAME}_class_library_test_plugins1 SHARED
    "${PROJECT_SOURCE_DIR}/autonomy/common/class_loader/test/plugins1_test.cpp")
list(APPEND plugin_libs ${PROJECT_NAME}_class_library_test_plugins1)

# TestPlugins2
add_library(${PROJECT_NAME}_class_library_test_plugins2 SHARED   
    "${PROJECT_SOURCE_DIR}/autonomy/common/class_loader/test/plugins2_test.cpp")
list(APPEND plugin_libs ${PROJECT_NAME}_class_library_test_plugins2)


################################ planner pugins #################################
# navfn_planner
add_library(navfn_planner SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/planning/plugins/navfn/navfn.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/plugins/navfn/navfn_planner.cpp"
)
list(APPEND plugin_libs navfn_planner)


#################################################################################

foreach(plugin ${plugin_libs})
    target_include_directories(${plugin}
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${plugin} ${PROJECT_NAME})
  
  # Set RPATH properties to avoid warnings - use $ORIGIN for relative paths
  set_target_properties(${plugin} PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH FALSE
    INSTALL_RPATH "\$ORIGIN/../lib:${CMAKE_INSTALL_PREFIX}/lib"
    BUILD_RPATH "\$ORIGIN:\$ORIGIN/../lib:${CMAKE_BINARY_DIR}/lib"
    SKIP_BUILD_RPATH FALSE
  )
endforeach()

# install
install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)