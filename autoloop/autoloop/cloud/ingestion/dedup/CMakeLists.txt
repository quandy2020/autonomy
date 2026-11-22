# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_dedup
  SOURCES
    dedup.cpp
    scene_clusterer.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_dedup_test
  SOURCES
    dedup_test.cpp
  DEPENDENCIES
    autoloop_cloud_dedup
)
