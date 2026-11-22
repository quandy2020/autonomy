# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_snapshot_writer
  SOURCES
    snapshot_writer.cpp
    autolink_writer_adapter.cpp
    mcap_exporter.cpp
  DEPENDENCIES
    autoloop_robot_ring_buffer
    autoloop_robot_trigger_manager
    autoloop_robot_version_stamp
    autoloop_common_base
)
autoloop_test(autoloop_robot_snapshot_writer_test
  SOURCES
    snapshot_writer_test.cpp
  DEPENDENCIES
    autoloop_robot_snapshot_writer
)
