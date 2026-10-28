cmake_minimum_required(VERSION 3.20)

project(atlas LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
option(ATLAS_BUILD_MESSAGE_ADAPTERS "Build optional AutoMsgs protocol conversions" OFF)
option(ATLAS_BUILD_APPLICATIONS "Build ROS-free SLAM applications" ON)
option(ATLAS_BUILD_VISUAL_SLAM "Build the ORB visual and visual-inertial SLAM pipeline" ON)

if(ATLAS_BUILD_MESSAGE_ADAPTERS)
  # Atlas keeps protobuf messages at the system boundary, while the SLAM core
  # uses its own PCL-free sensor types. Prefer an installed AutoMsgs package;
  # when building this checkout directly, use the adjacent source package.
  find_package(automsgs CONFIG QUIET)
  if(NOT TARGET automsgs::automsgs AND NOT TARGET automsgs::core AND NOT TARGET automsgs)
    set(_atlas_automsgs_source "${CMAKE_CURRENT_SOURCE_DIR}/../automsgs")
    if(NOT EXISTS "${_atlas_automsgs_source}/CMakeLists.txt")
      message(FATAL_ERROR "AutoMsgs is required. Install AutoMsgs or provide ../automsgs.")
    endif()
    set(AUTOMSGS_BUILD_TOOLS OFF)
    set(AUTOMSGS_BUILD_PYTHON OFF)
    set(AUTOMSGS_BUILD_EXAMPLES OFF)
    set(AUTOMSGS_BUILD_TESTS OFF)
    add_subdirectory("${_atlas_automsgs_source}" "${CMAKE_CURRENT_BINARY_DIR}/automsgs")
  endif()

  if(TARGET automsgs::automsgs)
    set(ATLAS_AUTOMSGS_TARGET automsgs::automsgs)
  elseif(TARGET automsgs::core)
    set(ATLAS_AUTOMSGS_TARGET automsgs::core)
  elseif(TARGET automsgs)
    set(ATLAS_AUTOMSGS_TARGET automsgs)
  else()
    message(FATAL_ERROR "AutoMsgs was found but did not export a usable CMake target.")
  endif()
endif()

add_subdirectory(atlas)
if(ATLAS_BUILD_APPLICATIONS)
  add_subdirectory(app)
endif()
