# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_telemetry
  SOURCES
    telemetry_collector.cpp
    metrics_publisher.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_telemetry_test
  SOURCES
    telemetry_collector_test.cpp
  DEPENDENCIES
    autoloop_robot_telemetry
)
