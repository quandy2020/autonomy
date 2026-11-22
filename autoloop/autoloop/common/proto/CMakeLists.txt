# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

autoloop_proto(autoloop_common_proto
  PROTOS
    ${AUTOLOOP_SRC_ROOT}/common/proto/version_tuple.proto
    ${AUTOLOOP_SRC_ROOT}/common/proto/trace_metadata.proto
    ${AUTOLOOP_SRC_ROOT}/common/proto/data_agent_health.proto)
