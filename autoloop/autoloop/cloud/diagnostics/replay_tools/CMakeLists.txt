# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_replay_tools
  SOURCES
    replay_tool.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_replay_tools_test
  SOURCES
    replay_tool_test.cpp
  DEPENDENCIES
    autoloop_cloud_replay_tools
)
