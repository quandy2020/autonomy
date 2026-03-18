# Copyright 2025 The Openbot Authors (duyongquan)
# Rendering library (Ogre/rviz-style helpers, etc.)

cmake_minimum_required(VERSION 3.16)

set(RENDERING_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB_RECURSE RENDERING_SOURCES "${RENDERING_DIR}/*.cpp")
file(GLOB_RECURSE RENDERING_HEADERS "${RENDERING_DIR}/*.hpp")

if(RENDERING_SOURCES)
  add_library(autoview_rendering STATIC ${RENDERING_SOURCES} ${RENDERING_HEADERS})
  target_include_directories(autoview_rendering PUBLIC
    "${RENDERING_DIR}/.."
    ${RENDERING_DIR}
  )
  target_link_libraries(autoview_rendering PUBLIC Qt5::Core Qt5::Widgets)
  set_target_properties(autoview_rendering PROPERTIES
    CXX_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
  )
  if(TARGET autoview)
    target_link_libraries(autoview PRIVATE autoview_rendering)
  endif()
else()
  message(STATUS "autoview/rendering: no sources yet, skipping autoview_rendering library")
endif()
