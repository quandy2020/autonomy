/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <string>

namespace autoviz {
namespace integration {
namespace grpc_client {

enum class MethodType {
  kUnary,
  kClientStreaming,
  kServerStreaming,
  kBidiStreaming,
};

struct GrpcStatusView {
  int code = 0;  // grpc_status_code
  std::string message;
  int64_t latency_ms = 0;
};

enum class StreamEventKind { kSent, kReceived, kError, kInfo };

struct StreamEvent {
  StreamEventKind kind;
  std::string json_or_text;
  int64_t timestamp_ms = 0;
};

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
