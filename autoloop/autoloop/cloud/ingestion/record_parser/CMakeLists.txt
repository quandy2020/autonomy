# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_record_parser
  SOURCES
    record_parser.cpp
    autolink_record_reader.cpp
    mcap_chunk_reader.cpp
    channel_extractor.cpp
  DEPENDENCIES
    autoloop_common_base
    autoloop_robot_ring_buffer
)
autoloop_test(autoloop_cloud_record_parser_test
  SOURCES
    record_parser_test.cpp
  DEPENDENCIES
    autoloop_cloud_record_parser
)
