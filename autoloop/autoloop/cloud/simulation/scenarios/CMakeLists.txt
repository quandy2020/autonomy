# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_scenarios
  SOURCES
    scenario_loader.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_scenarios_test
  SOURCES
    scenario_loader_test.cpp
  DEPENDENCIES
    autoloop_cloud_scenarios
)
