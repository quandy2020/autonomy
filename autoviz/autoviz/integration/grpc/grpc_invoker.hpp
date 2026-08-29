/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifndef AUTOVIZ_ENABLE_GRPC
#define AUTOVIZ_ENABLE_GRPC 0
#endif

#include <memory>
#include <string>

#if AUTOVIZ_ENABLE_GRPC
#include <grpcpp/channel.h>
#endif

#include "autoviz/integration/grpc/grpc_types.hpp"

namespace autoviz {
namespace integration {
namespace grpc_client {

class GrpcDescriptorStore;
class GrpcSession;

struct UnaryResult {
  std::string response_json;
  GrpcStatusView status;
  // Metadata deferred to Task 6.
};

class GrpcInvoker {
 public:
  explicit GrpcInvoker(GrpcDescriptorStore* store);

  /**
   * Sync unary RPC via GenericStub / BlockingUnaryCall + DynamicMessage.
   * |method_full_name|: package.Service.Method or package.Service/Method.
   */
  UnaryResult unaryCall(
#if AUTOVIZ_ENABLE_GRPC
      const std::shared_ptr<grpc::Channel>& channel,
#endif
      const std::string& method_full_name, const std::string& request_json,
      bool include_defaults, int timeout_ms, GrpcSession* session_for_cancel,
      std::string* err);

 private:
  GrpcDescriptorStore* store_ = nullptr;  // non-owning
};

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
