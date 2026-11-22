# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_controller
  SOURCES
    controller.cpp
    pure_pursuit_controller.cpp
    mpc_controller.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_controller_test
  SOURCES
    controller_test.cpp
  DEPENDENCIES
    autoloop_robot_controller
)
