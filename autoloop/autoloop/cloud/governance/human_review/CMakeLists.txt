# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_human_review
  SOURCES
    review_queue.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_human_review_test
  SOURCES
    review_queue_test.cpp
  DEPENDENCIES
    autoloop_cloud_human_review
)
