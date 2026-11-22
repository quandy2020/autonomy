# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_metadata_store
  SOURCES
    metadata_store.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_metadata_store_test
  SOURCES
    metadata_store_test.cpp
  DEPENDENCIES
    autoloop_cloud_metadata_store
)
