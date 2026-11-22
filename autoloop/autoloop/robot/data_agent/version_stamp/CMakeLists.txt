# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_version_stamp
  SOURCES
    version_stamp.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_version_stamp_test
  SOURCES
    version_stamp_test.cpp
  DEPENDENCIES
    autoloop_robot_version_stamp
)
