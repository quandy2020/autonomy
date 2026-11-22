# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.
autoloop_library(autoloop_cloud_telemetry_gate
  SOURCES
    telemetry_gate.cpp
  DEPENDENCIES
    autoloop_common_base
)
autoloop_test(autoloop_cloud_telemetry_gate_test
  SOURCES
    telemetry_gate_test.cpp
  DEPENDENCIES
    autoloop_cloud_telemetry_gate
)
