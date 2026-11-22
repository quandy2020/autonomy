# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_camera
  SOURCES
    camera_driver.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_camera_test
  SOURCES
    camera_driver_test.cpp
  DEPENDENCIES
    autoloop_robot_camera
)
