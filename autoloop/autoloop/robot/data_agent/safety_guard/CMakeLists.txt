# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_safety_guard
  SOURCES
    safety_guard.cpp
)
autoloop_test(autoloop_robot_safety_guard_test
  SOURCES
    safety_guard_test.cpp
  DEPENDENCIES
    autoloop_robot_safety_guard
)
