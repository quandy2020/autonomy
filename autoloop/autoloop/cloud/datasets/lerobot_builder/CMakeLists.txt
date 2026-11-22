# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_lerobot_builder
  SOURCES
    dataset_builder.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_lerobot_builder_test
  SOURCES
    dataset_builder_test.cpp
  DEPENDENCIES
    autoloop_cloud_lerobot_builder
)
