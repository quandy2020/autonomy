# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_monitor_bridge
  SOURCES
    monitor_bridge.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_monitor_bridge_test
  SOURCES
    monitor_bridge_test.cpp
  DEPENDENCIES
    autoloop_cloud_monitor_bridge
)
