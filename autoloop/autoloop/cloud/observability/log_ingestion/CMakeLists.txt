# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_log_ingestion
  SOURCES
    log_ingestion.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_log_ingestion_test
  SOURCES
    log_ingestion_test.cpp
  DEPENDENCIES
    autoloop_cloud_log_ingestion
)
