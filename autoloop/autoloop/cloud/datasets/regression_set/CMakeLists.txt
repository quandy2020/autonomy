# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_regression_set
  SOURCES
    regression_set.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_regression_set_test
  SOURCES
    regression_set_test.cpp
  DEPENDENCIES
    autoloop_cloud_regression_set
)
