# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.


add_subdirectory(ring_buffer)
add_subdirectory(trigger_manager)
add_subdirectory(version_stamp)
add_subdirectory(snapshot_writer)
add_subdirectory(structured_log)
add_subdirectory(telemetry)
add_subdirectory(uploader)
add_subdirectory(safety_guard)

autoloop_library(autoloop_robot_data_agent
  SOURCES
    data_agent_component.cpp
  DEPENDENCIES
    autoloop_robot_ring_buffer
    autoloop_robot_trigger_manager
    autoloop_robot_snapshot_writer
    autoloop_robot_uploader
    autoloop_robot_version_stamp
    autoloop_robot_safety_guard
    autoloop_robot_telemetry
    autoloop_robot_structured_log
    autoloop_common_base)
autoloop_test(autoloop_robot_data_agent_test
  SOURCES
    data_agent_component_test.cpp
  DEPENDENCIES
    autoloop_robot_data_agent)
