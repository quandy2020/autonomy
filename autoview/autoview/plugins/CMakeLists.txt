# Copyright 2025 The Openbot Authors (duyongquan)
# Plugins: view_controllers, transformation, etc.

cmake_minimum_required(VERSION 3.16)

set(PLUGINS_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

set(PLUGINS_SOURCES
  view_controllers/follower/third_person_follower_view_controller.cpp
  view_controllers/fps/fps_view_controller.cpp
  transformation/tf_wrapper.cpp
)
set(PLUGINS_HEADERS
  view_controllers/follower/third_person_follower_view_controller.hpp
  view_controllers/fps/fps_view_controller.hpp
  transformation/tf_wrapper.hpp
)

add_library(autoview_plugins STATIC ${PLUGINS_SOURCES} ${PLUGINS_HEADERS})
target_include_directories(autoview_plugins PUBLIC
  "${PLUGINS_DIR}/.."
  ${PLUGINS_DIR}
)
target_link_libraries(autoview_plugins PUBLIC Qt5::Core Qt5::Widgets)
set_target_properties(autoview_plugins PROPERTIES
  CXX_STANDARD 17
  POSITION_INDEPENDENT_CODE ON
)

if(TARGET autoview)
  target_link_libraries(autoview PRIVATE autoview_plugins)
endif()
