# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_planner
  SOURCES
    planner.cpp
    astar_planner.cpp
    behavior_tree.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_planner_test
  SOURCES
    planner_test.cpp
  DEPENDENCIES
    autoloop_robot_planner
)
