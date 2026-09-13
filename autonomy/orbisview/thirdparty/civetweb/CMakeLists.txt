# Copyright 2026 The Openbot Authors
# Vendored CivetWeb v1.16 — HTTP + WebSocket for OrbisView only.

cmake_minimum_required(VERSION 3.20)

add_library(orbisview_civetweb STATIC
  src/civetweb.c
  src/CivetServer.cpp
)
add_library(orbisview::civetweb ALIAS orbisview_civetweb)

target_include_directories(orbisview_civetweb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(orbisview_civetweb PUBLIC
  USE_WEBSOCKET
  NO_SSL
  NO_CGI
  NO_CACHING
)

find_package(Threads REQUIRED)
target_link_libraries(orbisview_civetweb PUBLIC Threads::Threads)

if(UNIX AND NOT APPLE)
  target_link_libraries(orbisview_civetweb PUBLIC dl)
endif()

# civetweb.c is C; ensure C++ linker for CivetServer.cpp
set_target_properties(orbisview_civetweb PROPERTIES
  LINKER_LANGUAGE CXX
  POSITION_INDEPENDENT_CODE ON)

# Silence noisy third-party warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(orbisview_civetweb PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>)
endif()
