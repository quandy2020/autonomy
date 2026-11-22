# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_model_sync
  SOURCES
    model_sync.cpp
    model_fetcher.cpp
    local_registry.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_model_sync_test
  SOURCES
    model_sync_test.cpp
  DEPENDENCIES
    autoloop_robot_model_sync
)
