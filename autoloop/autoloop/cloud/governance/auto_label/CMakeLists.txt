# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_auto_label
  SOURCES
    auto_labeler.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_auto_label_test
  SOURCES
    auto_labeler_test.cpp
  DEPENDENCIES
    autoloop_cloud_auto_label
)
