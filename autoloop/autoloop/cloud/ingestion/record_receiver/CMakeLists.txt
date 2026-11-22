# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_record_receiver
  SOURCES
    record_receiver.cpp
    hash_verifier.cpp
    archive_store.cpp
  DEPENDENCIES
    autoloop_common_base
    autoloop_robot_version_stamp
)
autoloop_test(autoloop_cloud_record_receiver_test
  SOURCES
    record_receiver_test.cpp
  DEPENDENCIES
    autoloop_cloud_record_receiver
)
