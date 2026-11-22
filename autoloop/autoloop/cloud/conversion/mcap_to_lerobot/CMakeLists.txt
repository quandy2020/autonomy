# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_mcap_to_lerobot
  SOURCES
    converter.cpp
    field_mapping.cpp
    clock_alignment.cpp
    video_encoder.cpp
    parquet_writer.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_mcap_to_lerobot_test
  SOURCES
    converter_test.cpp
  DEPENDENCIES
    autoloop_cloud_mcap_to_lerobot
)
