# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

add_subdirectory(proto)

autoloop_library(autoloop_robot_recorder
  SOURCES
    bt_attribution.cpp
    clock.cpp
    convert.cpp
    episode.cpp
    event_graph.cpp
    hash_chain.cpp
    mcap_writer.cpp
    metrics.cpp
    observation.cpp
    pipeline.cpp
    player.cpp
    recorder.cpp
    replay.cpp
    rolling_buffer.cpp
    root_cause.cpp
    safety_latch.cpp
    safety_monitor.cpp
    sidecar.cpp
    signals.cpp
    subscriber.cpp
    trigger.cpp
    upload.cpp
    value_tier.cpp
    version_tuple.cpp
  DEPENDENCIES
    autoloop_robot_recorder_proto
    autoloop_json)
if(TARGET autoloop_autolink_headers)
  target_link_libraries(autoloop_robot_recorder PUBLIC autoloop_autolink_headers)
endif()
if(TARGET automsgs)
  target_link_libraries(autoloop_robot_recorder PUBLIC automsgs)
endif()
if(TARGET glog::glog)
  target_link_libraries(autoloop_robot_recorder PUBLIC glog::glog)
endif()

option(AUTOLOOP_BUILD_RECORDER "Build autoloop.recorder binary" ON)
if(AUTOLOOP_BUILD_RECORDER)
  add_executable(autoloop.recorder recorder_main.cpp)
  target_link_libraries(autoloop.recorder PRIVATE
    autoloop_robot_recorder
    autoloop_cli11)
  target_include_directories(autoloop.recorder PRIVATE
    ${AUTOLOOP_SRC_ROOT} ${AUTOLOOP_GEN_DIR})
endif()

autoloop_test(autoloop_robot_recorder_test
  SOURCES recorder_test.cpp
  DEPENDENCIES autoloop_robot_recorder)
