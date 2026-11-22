# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_trace_index
  SOURCES
    trace_index.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_trace_index_test
  SOURCES
    trace_index_test.cpp
  DEPENDENCIES
    autoloop_cloud_trace_index
)
