# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.


add_executable(autoloop-convert
  main.cpp convert_command.cpp check_command.cpp)
target_link_libraries(autoloop-convert PRIVATE
  autoloop_cloud_mcap_to_lerobot
  autoloop_cloud_record_receiver
  autoloop_common_base)
target_include_directories(autoloop-convert PRIVATE
  ${AUTOLOOP_SRC_ROOT} ${AUTOLOOP_GEN_DIR})
