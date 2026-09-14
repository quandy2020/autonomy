# Copyright 2026 The Openbot Authors
#
# OrbisView — CivetWeb HTTP/WS + mock/autolink adapters + frontend (npm).
# Layout: backend / frontend / proto / conf / launch / main.cc
# thirdparty/civetweb IS built. Apollo Dreamview sources are not vendored.

cmake_minimum_required(VERSION 3.20)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(orbisview LANGUAGES C CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(glog REQUIRED)
  find_package(gflags REQUIRED)
  option(BUILD_TEST "Build OrbisView unit tests" ON)
  if(BUILD_TEST)
    enable_testing()
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
      include(FetchContent)
      FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
      set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googletest)
    endif()
  endif()
endif()

option(ORBISVIEW_WITH_AUTOLINK
  "Link OrbisView against Autolink for live channel discovery"
  OFF)

set(_ORBISVIEW_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(_ORBISVIEW_INC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_subdirectory(thirdparty/civetweb)
add_subdirectory(backend)
add_subdirectory(proto)

set(_ORBISVIEW_SRCS
  backend/common/stream_envelope.cc
  backend/simulation_world/simulation_world_service.cc
  backend/simulation_world/simulation_world_updater.cc
  backend/hmi/hmi_worker.cc
  backend/common/handlers/websocket_handler.cc
  backend/common/orbisview_gflags.cc
  backend/teleop/teleop.cc
  backend/orbisview.cc
  backend/record_player/envelope_recorder.cc
  backend/adapters/mock/mock_source.cc
  backend/common/plugins/plugin_host.cc
)

add_library(orbisview_lib STATIC ${_ORBISVIEW_SRCS})
add_library(autonomy::orbisview_lib ALIAS orbisview_lib)
target_include_directories(orbisview_lib PUBLIC
  $<BUILD_INTERFACE:${_ORBISVIEW_INC_ROOT}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(orbisview_lib PUBLIC cxx_std_17)
find_package(Threads REQUIRED)
target_link_libraries(orbisview_lib PUBLIC
  Threads::Threads
  ${CMAKE_DL_LIBS}
  orbisview_civetweb)

if(TARGET glog::glog)
  target_link_libraries(orbisview_lib PUBLIC glog::glog)
elseif(TARGET glog)
  target_link_libraries(orbisview_lib PUBLIC glog)
else()
  find_package(glog REQUIRED)
  target_link_libraries(orbisview_lib PUBLIC glog::glog)
endif()

if(TARGET gflags::gflags)
  target_link_libraries(orbisview_lib PUBLIC gflags::gflags)
elseif(TARGET gflags)
  target_link_libraries(orbisview_lib PUBLIC gflags)
endif()

set(_ORBISVIEW_HAS_AUTOMSGS OFF)
if(ORBISVIEW_WITH_AUTOLINK AND TARGET autolink)
  target_compile_definitions(orbisview_lib PUBLIC ORBISVIEW_WITH_AUTOLINK=1)
  target_link_libraries(orbisview_lib PUBLIC autolink)
  if(TARGET automsgs OR TARGET automsgs_proto)
    set(_ORBISVIEW_HAS_AUTOMSGS ON)
    target_compile_definitions(orbisview_lib PUBLIC ORBISVIEW_WITH_AUTOMSGS=1)
    target_sources(orbisview_lib PRIVATE
      backend/adapters/automsgs/automsgs_converter.cc)
    if(TARGET automsgs)
      target_link_libraries(orbisview_lib PUBLIC automsgs)
    else()
      target_link_libraries(orbisview_lib PUBLIC automsgs_proto)
    endif()
    message(STATUS "orbisview: Autolink + automsgs adapters ON")
  else()
    message(STATUS "orbisview: Autolink ON (raw only; no automsgs)")
  endif()
else()
  message(STATUS "orbisview: Autolink integration OFF (mock-only)")
endif()

add_library(autonomy_orbisview INTERFACE)
add_library(autonomy::orbisview ALIAS autonomy_orbisview)
target_link_libraries(autonomy_orbisview INTERFACE orbisview_lib)

add_executable(autonomy.orbisview main.cc)
target_link_libraries(autonomy.orbisview PRIVATE orbisview_lib)
if(TARGET gflags)
  target_link_libraries(autonomy.orbisview PRIVATE gflags)
elseif(TARGET gflags::gflags)
  target_link_libraries(autonomy.orbisview PRIVATE gflags::gflags)
endif()
set_target_properties(autonomy.orbisview PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
install(TARGETS autonomy.orbisview RUNTIME DESTINATION bin)

add_library(orbisview_example_hello MODULE
  backend/common/plugins/examples/example_hello_plugin.cc)
target_include_directories(orbisview_example_hello PRIVATE
  ${_ORBISVIEW_INC_ROOT})
set_target_properties(orbisview_example_hello PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/orbisview/plugins"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/orbisview/plugins")
add_dependencies(autonomy.orbisview orbisview_example_hello)

add_custom_target(orbisview_frontend_hint
  COMMAND ${CMAKE_COMMAND} -E echo
    "OrbisView: npm run dev in frontend/ or serve dist via --document_root"
  VERBATIM)
add_custom_target(orbisview_all DEPENDS autonomy.orbisview orbisview_frontend_hint)

if(BUILD_TEST AND TARGET GTest::gtest_main)
  add_executable(orbisview_throttle_queue_test
    backend/common/streaming/throttle_queue_test.cc)
  target_link_libraries(orbisview_throttle_queue_test
    PRIVATE orbisview_lib GTest::gtest_main)
  add_test(NAME orbisview_throttle_queue_test
    COMMAND orbisview_throttle_queue_test)

  add_executable(orbisview_stream_envelope_test
    backend/common/stream_envelope_test.cc)
  target_link_libraries(orbisview_stream_envelope_test
    PRIVATE orbisview_lib GTest::gtest_main)
  add_test(NAME orbisview_stream_envelope_test
    COMMAND orbisview_stream_envelope_test)

  add_executable(orbisview_envelope_recorder_test
    backend/record_player/envelope_recorder_test.cc)
  target_link_libraries(orbisview_envelope_recorder_test
    PRIVATE orbisview_lib GTest::gtest_main)
  add_test(NAME orbisview_envelope_recorder_test
    COMMAND orbisview_envelope_recorder_test)

  add_executable(orbisview_plugin_host_test
    backend/common/plugins/plugin_host_test.cc)
  target_link_libraries(orbisview_plugin_host_test
    PRIVATE orbisview_lib GTest::gtest_main)
  add_test(NAME orbisview_plugin_host_test
    COMMAND orbisview_plugin_host_test)

  if(_ORBISVIEW_HAS_AUTOMSGS)
    add_executable(orbisview_automsgs_converter_test
      backend/adapters/automsgs/automsgs_converter_test.cc)
    target_link_libraries(orbisview_automsgs_converter_test
      PRIVATE orbisview_lib GTest::gtest_main)
    add_test(NAME orbisview_automsgs_converter_test
      COMMAND orbisview_automsgs_converter_test)
  endif()
endif()

message(STATUS "autonomy: OrbisView CivetWeb stack enabled")
