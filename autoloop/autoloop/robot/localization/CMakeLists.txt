# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_localization
  SOURCES
    fusion_localizer.cpp
    odometry_fusion.cpp
    map_matcher.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_localization_test
  SOURCES
    fusion_localizer_test.cpp
  DEPENDENCIES
    autoloop_robot_localization
)
