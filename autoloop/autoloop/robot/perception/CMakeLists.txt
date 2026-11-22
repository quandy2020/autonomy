# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_perception
  SOURCES
    obstacle_detector.cpp
    semantic_segmenter.cpp
    tracker.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_robot_perception_test
  SOURCES
    obstacle_detector_test.cpp
  DEPENDENCIES
    autoloop_robot_perception
)
