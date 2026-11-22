# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_robot_uploader
  SOURCES
    uploader.cpp
    upload_queue.cpp
    resumable_upload.cpp
  DEPENDENCIES
    autoloop_robot_trigger_manager
    autoloop_common_base
)
autoloop_test(autoloop_robot_uploader_test
  SOURCES
    uploader_test.cpp
  DEPENDENCIES
    autoloop_robot_uploader
)
