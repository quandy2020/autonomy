# Copyright 2026 The Openbot Authors (duyongquan)
#
# Autoviz — standalone Qt visualizer; depends on sibling autolink + automsgs only.
#
#   cmake -B build && cmake --build build --target autoviz_app
#   libautoviz (autoviz::core) + thin bin/autoviz (target autoviz_app)

cmake_minimum_required(VERSION 3.20)
project(autoviz VERSION 0.1.0 LANGUAGES CXX C)

option(AUTOVIZ_BUILD_WIDGET_TESTS "Build the Autoviz widget QtTest suite" OFF)
option(AUTOVIZ_BUILD_CORE_TESTS "Build the Autoviz core + headless UI test suite" OFF)
option(AUTOVIZ_BUILD_WIDGET_GALLERY
  "Build the Autoviz widget component gallery" OFF)

find_package(autocmake CONFIG QUIET)
if(NOT autocmake_FOUND)
  set(AUTOCMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autocmake")
  include("${AUTOCMAKE_DIR}/Autocmake.cmake")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Functions.cmake")
autoviz_setup()
autoviz_embed_siblings()

autocmake_project()
autocmake_find(Qt6 REQUIRED PRIVATE
  COMPONENTS Core Gui Widgets OpenGLWidgets OpenGL Xml Svg Network)
autocmake_build(QUIT)

autoviz_collect_sources()
autocmake_library(autoviz SHARED
  NO_EXPORT
  SOURCES
    ${AUTOVIZ_SOURCES}
    ${AUTOVIZ_HEADERS}
    ${AUTOVIZ_RECORDER_SOURCES}
    ${AUTOVIZ_ROOT}/resources/autoviz.qrc
    ${AUTOVIZ_ROOT}/styles/styles.qrc
  INCLUDES ${AUTOVIZ_ROOT}
  DEPENDENCIES automsgs autolink)
autoviz_finalize_library(autoviz)

if(TARGET autonomy_proto AND AUTONOMY_BUILD_LOCALIZATION)
  find_package(ZLIB REQUIRED)
  target_compile_definitions(autoviz PRIVATE AUTOVIZ_HAS_CARTOGRAPHER=1)
  target_include_directories(autoviz PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})
  target_link_libraries(autoviz PRIVATE autonomy_proto ZLIB::ZLIB)
endif()

autocmake_binary(autoviz_app
  NO_EXPORT
  OUTPUT_NAME autoviz
  SOURCES ${AUTOVIZ_SRC_ROOT}/main.cpp
  DEPENDENCIES autoviz)
if(AUTONOMY_BUILD_COVERAGE)
  target_compile_definitions(autoviz PRIVATE AUTONOMY_BUILD_COVERAGE=1)
  target_link_libraries(autoviz_app PRIVATE autonomy_proto)
  target_include_directories(autoviz PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR})
  target_link_libraries(autoviz PRIVATE autonomy_proto)
endif()
target_link_libraries(autoviz_app PRIVATE
  Qt6::Core Qt6::Gui Qt6::Widgets glog::glog)

if(AUTOVIZ_BUILD_WIDGET_TESTS OR AUTOVIZ_BUILD_CORE_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(AUTOVIZ_BUILD_WIDGET_GALLERY)
  add_executable(autoviz_widget_gallery
    examples/widget_gallery_main.cpp)
  target_include_directories(autoviz_widget_gallery PRIVATE ${AUTOVIZ_ROOT})
  target_link_libraries(autoviz_widget_gallery PRIVATE
    autoviz Qt6::Core Qt6::Gui Qt6::Widgets)
endif()

autoviz_install_desktop_assets()
autocmake_package()
autoviz_print_summary()
