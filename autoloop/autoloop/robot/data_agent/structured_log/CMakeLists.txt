# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_structured_log
  SOURCES
    glog_config.cpp
    custom_prefix.cpp
    trace_context.cpp
    event_writer.cpp
    crash_handler.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_structured_log_test
  SOURCES
    structured_log_test.cpp
  DEPENDENCIES
    autoloop_robot_structured_log
)
