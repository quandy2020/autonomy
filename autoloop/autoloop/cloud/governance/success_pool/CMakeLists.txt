# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_success_pool
  SOURCES
    success_pool.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_success_pool_test
  SOURCES
    success_pool_test.cpp
  DEPENDENCIES
    autoloop_cloud_success_pool
)
