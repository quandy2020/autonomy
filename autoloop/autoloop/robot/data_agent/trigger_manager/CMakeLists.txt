# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_trigger_manager
  SOURCES
    trigger_manager.cpp
    trigger_rule.cpp
  DEPENDENCIES
    autoloop_robot_ring_buffer
    autoloop_common_base
)
autoloop_test(autoloop_robot_trigger_manager_test
  SOURCES
    trigger_manager_test.cpp
  DEPENDENCIES
    autoloop_robot_trigger_manager
)
