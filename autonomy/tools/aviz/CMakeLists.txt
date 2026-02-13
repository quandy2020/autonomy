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

cmake_minimum_required(VERSION 3.8)

# aviz visualizer has been disabled and its sources removed from this tree.
# Keep this CMakeLists.txt as a stub so that parent CMakeLists can still
# add_subdirectory() without causing configuration errors.
message(STATUS "aviz visualizer disabled: skipping build in autonomy/tools/aviz")
return()

# Enable Qt5 AUTOMOC, AUTOUIC, and AUTORCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find dependencies
# Note: autolink should be available from parent CMakeLists.txt
# Try to use autolink target directly (same project), or find_package if installed
if(NOT TARGET autolink::autolink)
    if(TARGET autolink)
        # Create alias for autolink target to match expected interface
        add_library(autolink::autolink ALIAS autolink)
    else()
        find_package(autolink REQUIRED)
    endif()
endif()

# FastDDS and Protobuf should already be found by autolink
# Use the same approach as autolink
find_package(FastDDS REQUIRED)
if(FastDDS_FOUND)
    if(TARGET FastDDS::fastcdr AND TARGET FastDDS::fastrtps)
        # Use modern target names
        set(FASTDDS_TARGETS FastDDS::fastcdr FastDDS::fastrtps)
    else()
        # Fall back to library variables (as used by autolink)
        set(FASTDDS_TARGETS ${FastDDS_LIBRARIES})
    endif()
else()
    message(FATAL_ERROR "FastDDS not found")
endif()

find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# Find Ogre (REQUIRED for rendering module)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake/modules")
find_package(Ogre REQUIRED)
if(NOT OGRE_FOUND)
    message(FATAL_ERROR "Ogre is REQUIRED for the rendering module but was not found.")
endif()
message(STATUS "Found Ogre: ${OGRE_INCLUDE_DIRS} ${OGRE_LIBRARIES}")

# Collect source files
file(GLOB AVIZ_SRCS
    "*.cpp"
    "plugins/*.cpp"
    "common/*.cpp"
    "common/properties/*.cpp"
    "common/interaction/*.cpp"
    "common/transformation/*.cpp"
    "rendering/*.cpp"
    "rendering/objects/*.cpp"
    "plugins/displays/*/*.cpp"
    "plugins/tools/*/*.cpp"
    "plugins/view_controllers/*/*.cpp"
    "plugins/robot/*.cpp"
    "plugins/transformation/*.cpp"
)

file(GLOB AVIZ_HDRS
    "*.hpp"
    "plugins/*.hpp"
    "common/*.hpp"
    "rendering/*.hpp"
    "rendering/objects/*.hpp"
)

# Collect Qt resource files
file(GLOB AVIZ_QRC
    "asserts/*.qrc"
)

# Collect Qt UI files
file(GLOB AVIZ_UIS
    "uis/*.ui"
)

# Wrap UI files to generate header files
set(AVIZ_UI_HEADERS)
qt5_wrap_ui(AVIZ_UI_HEADERS ${AVIZ_UIS})

# Ensure UI files are processed by AUTOUIC
set_source_files_properties(${AVIZ_UIS} PROPERTIES AUTOUIC ON)

# Collect resource directories (images and shaders)
set(AVIZ_RESOURCE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/images"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders"
)

# Copy resource directories to build directory (for runtime access)
foreach(RES_DIR ${AVIZ_RESOURCE_DIRS})
    if(EXISTS ${RES_DIR})
        file(COPY ${RES_DIR} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endforeach()

# Create the executable
add_executable(aviz
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${AVIZ_SRCS}
    ${AVIZ_HDRS}
    ${AVIZ_QRC}
    ${AVIZ_UIS}
    ${AVIZ_UI_HEADERS}
)

# Set target properties for Qt5
set_target_properties(aviz PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)

# Ensure AUTOUIC can find .ui files inside the uis/ directory
set_target_properties(aviz PROPERTIES
    AUTOUIC_SEARCH_PATHS "uis"
)

# Include directories for executable
# Add project source root to include path so autonomy/... paths work
target_include_directories(aviz
    PUBLIC
        ${PROJECT_SOURCE_DIR}        # For autonomy/... includes
        ${CMAKE_CURRENT_SOURCE_DIR}  # For local aviz headers (e.g. treewidget.h)
        ${CMAKE_CURRENT_BINARY_DIR}  # For Qt UI generated headers
    PRIVATE
        ${PROJECT_BINARY_DIR}  # For proto generated headers
        ${EIGEN3_INCLUDE_DIR}  # For Eigen
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
)

# Get FastDDS include directories if available
if(FastDDS_FOUND)
    if(FastDDS_INCLUDE_DIRS)
        target_include_directories(aviz PRIVATE ${FastDDS_INCLUDE_DIRS})
    endif()
endif()

# Find OpenGL (required for glPointSize and other OpenGL functions)
find_package(OpenGL REQUIRED)

# Add Ogre include directories and libraries (after add_executable)
# Ogre headers are in /usr/include/OGRE, but Overlay headers use relative paths
# So we need to add /usr/include/OGRE to the include path for relative includes
if(OGRE_INCLUDE_DIRS)
    target_include_directories(aviz PRIVATE ${OGRE_INCLUDE_DIRS})
    # Also add OGRE subdirectory for relative includes in Overlay headers
    list(GET OGRE_INCLUDE_DIRS 0 OGRE_BASE_DIR)
    if(OGRE_BASE_DIR)
        target_include_directories(aviz PRIVATE "${OGRE_BASE_DIR}/OGRE")
    endif()
endif()

# Link the library to the executable
target_link_libraries(aviz
    PRIVATE
        autolink::autolink
        ${FASTDDS_TARGETS}
        ${Protobuf_LIBRARIES}
        Threads::Threads
        autonomy  # Link autonomy library for protobuf definitions (RadarObstacles, Point2D, etc.)
        urdfdom_headers  # URDF parsing headers
        urdf_parser  # URDF parsing library
        Qt5::Core
        Qt5::Gui
        Qt5::OpenGL
        Qt5::Widgets
        OpenGL::GL  # Link OpenGL library for glPointSize and other functions
        ${OGRE_LIBRARIES}  # Link Ogre libraries
        OgreOverlay  # Link OgreOverlay library for OverlaySystem
)

# Add compile and link options for the executable
target_compile_options(aviz PRIVATE
    -pthread
)

target_link_options(aviz PRIVATE
    -pthread
)

# Set C++ standard
set_target_properties(aviz PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install targets
install(TARGETS
    aviz
    DESTINATION bin
)

# Install resource directories
# Only install if directories exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/asserts/images)
    install(DIRECTORY asserts/images/
        DESTINATION share/${PROJECT_NAME}/aviz/images
        FILES_MATCHING PATTERN "*"
    )
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/asserts/shaders)
    install(DIRECTORY asserts/shaders/
        DESTINATION share/${PROJECT_NAME}/aviz/shaders
        FILES_MATCHING PATTERN "*"
    )
endif()

# Install plugin description file
install(FILES plugins/plugins_description.xml
    DESTINATION share/${PROJECT_NAME}/aviz/plugins
)
