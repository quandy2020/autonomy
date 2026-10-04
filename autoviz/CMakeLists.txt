# Copyright 2026 The Openbot Authors (duyongquan)
#
# Autoviz — standalone Qt visualizer; depends on sibling autolink + automsgs only.
#
#   cmake -B build && cmake --build build --target autoviz_app
#   libautoviz (autoviz::core) + thin bin/autoviz (target autoviz_app)

cmake_minimum_required(VERSION 3.20)
project(autoviz VERSION 0.1.0 LANGUAGES CXX C)

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

autocmake_binary(autoviz_app
  NO_EXPORT
  OUTPUT_NAME autoviz
  SOURCES ${AUTOVIZ_SRC_ROOT}/main.cpp
  DEPENDENCIES autoviz)
target_link_libraries(autoviz_app PRIVATE
  Qt6::Core Qt6::Gui Qt6::Widgets glog::glog)

autoviz_install_desktop_assets()
autocmake_package()
autoviz_print_summary()
