# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_lidar
  SOURCES
    lidar_driver.cpp
    lidar_config.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_lidar_test
  SOURCES
    lidar_driver_test.cpp
  DEPENDENCIES
    autoloop_robot_lidar
)
