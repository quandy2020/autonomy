# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_model_publisher
  SOURCES
    model_publisher.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_model_publisher_test
  SOURCES
    model_publisher_test.cpp
  DEPENDENCIES
    autoloop_cloud_model_publisher
)
