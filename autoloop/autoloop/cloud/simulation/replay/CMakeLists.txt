# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_replay
  SOURCES
    replay_controller.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_replay_test
  SOURCES
    replay_controller_test.cpp
  DEPENDENCIES
    autoloop_cloud_replay
)
