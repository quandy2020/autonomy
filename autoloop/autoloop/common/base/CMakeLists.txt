# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

autoloop_library(autoloop_common_base
  SOURCES
    hash_util.cpp
    json_util.cpp
    time_util.cpp
    path_util.cpp
  DEPENDENCIES
    autoloop_common_proto)
if(TARGET autoloop_autolink_headers)
  target_link_libraries(autoloop_common_base PUBLIC autoloop_autolink_headers)
endif()

autoloop_test(autoloop_common_base_test
  SOURCES
    hash_util_test.cpp
    json_util_test.cpp
    time_util_test.cpp
    path_util_test.cpp
    status_test.cpp
  DEPENDENCIES
    autoloop_common_base)
