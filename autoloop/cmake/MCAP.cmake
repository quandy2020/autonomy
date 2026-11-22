# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

# Foxglove MCAP C++ SDK (submodule thirdparty/mcap). Header-only.
find_path(MCAP_INCLUDE_DIR mcap/writer.hpp
  HINTS "${CMAKE_CURRENT_LIST_DIR}/../thirdparty/mcap/cpp/mcap/include")
if(MCAP_INCLUDE_DIR)
  add_library(autoloop_mcap INTERFACE)
  target_include_directories(autoloop_mcap INTERFACE ${MCAP_INCLUDE_DIR})
  target_compile_definitions(autoloop_mcap INTERFACE
    MCAP_COMPRESSION_NO_LZ4
    MCAP_COMPRESSION_NO_ZSTD)
  set(AUTOLOOP_WITH_MCAP_SDK ON)
  message(STATUS "Autoloop: mcap headers at ${MCAP_INCLUDE_DIR}")
else()
  set(AUTOLOOP_WITH_MCAP_SDK OFF)
  message(STATUS "Autoloop: mcap SDK not found")
endif()
