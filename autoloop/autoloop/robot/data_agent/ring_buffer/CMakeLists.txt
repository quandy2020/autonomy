# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_ring_buffer
  SOURCES
    ring_buffer.cpp
    ring_buffer_config.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_ring_buffer_test
  SOURCES
    ring_buffer_test.cpp
  DEPENDENCIES
    autoloop_robot_ring_buffer
)
