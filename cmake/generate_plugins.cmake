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

# NOTE:
# This file is included from the top-level `CMakeLists.txt`. It is expected to run on every configure.
# Do NOT cache a "PLUGINS_CREATED" flag, otherwise an existing build directory may skip plugin target
# creation on subsequent configures, causing missing `TARGET` errors in downstream subdirectories.

# Initialize plugin_libs list
set(plugin_libs "")

################################ map plugins ####################################
include(${PROJECT_SOURCE_DIR}/cmake/plugins/map_plugins.cmake)

################################ planner plugins ################################
include(${PROJECT_SOURCE_DIR}/cmake/plugins/planner_plugins.cmake)

################################ navigator plugins ################################
include(${PROJECT_SOURCE_DIR}/cmake/plugins/navigator_plugins.cmake)

################################ controller plugins ################################
include(${PROJECT_SOURCE_DIR}/cmake/plugins/controller_plugins.cmake)

# ################################ behavior tree plugins ############################
include(${PROJECT_SOURCE_DIR}/cmake/plugins/behavior_tree_plugins.cmake)

#################################################################################

# Collect all plugin source and header files
set(ALL_PLUGINS_SRCS "")
set(ALL_PLUGINS_HDRS "")

foreach(plugin ${plugin_libs})
    # Get source files for this plugin
    get_target_property(plugin_sources ${plugin} SOURCES)
    if(plugin_sources)
        foreach(src ${plugin_sources})
            # Convert to absolute path for matching with GLOB_RECURSE results
            get_filename_component(abs_src ${src} ABSOLUTE BASE_DIR ${PROJECT_SOURCE_DIR})
            list(APPEND ALL_PLUGINS_SRCS ${abs_src})
            # Convert .cpp to .hpp for corresponding header file
            string(REPLACE ".cpp" ".hpp" hpp_file ${src})
            get_filename_component(abs_hpp ${hpp_file} ABSOLUTE BASE_DIR ${PROJECT_SOURCE_DIR})
            if(EXISTS ${abs_hpp})
                list(APPEND ALL_PLUGINS_HDRS ${abs_hpp})
            endif()
        endforeach()
    endif()
endforeach()

# Export plugin files to parent scope so they can be used in CMakeLists.txt
set(ALL_PLUGINS_SRCS "${ALL_PLUGINS_SRCS}" PARENT_SCOPE)
set(ALL_PLUGINS_HDRS "${ALL_PLUGINS_HDRS}" PARENT_SCOPE)

foreach(plugin ${plugin_libs})
    target_include_directories(${plugin}
    PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${plugin} PUBLIC ${PROJECT_NAME})
  
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