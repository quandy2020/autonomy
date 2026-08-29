/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifndef AUTOVIZ_ENABLE_GRPC
#define AUTOVIZ_ENABLE_GRPC 0
#endif

#include <memory>
#include <mutex>
#include <string>

#if AUTOVIZ_ENABLE_GRPC
#include <grpcpp/grpcpp.h>
#endif

namespace autoviz {
namespace integration {
namespace grpc_client {

/** Strip grpc:// / http:// / https:// prefixes; keep host:port. */
std::string NormalizeGrpcTarget(const std::string& target);

/** Normalize package.Service.Method vs package.Service/Method for FindMethod. */
std::string NormalizeMethodFullName(const std::string& method_full_name);

class GrpcSession {
 public:
  GrpcSession() = default;
  ~GrpcSession();

  GrpcSession(const GrpcSession&) = delete;
  GrpcSession& operator=(const GrpcSession&) = delete;

  bool connect(const std::string& target, bool use_tls, bool verify_cert,
               const std::string& ssl_override, std::string* err);

#if AUTOVIZ_ENABLE_GRPC
  std::shared_ptr<grpc::Channel> channel() const { return channel_; }

  /** Register the ClientContext currently in flight (invoker sets; cancel clears). */
  void setActiveContext(grpc::ClientContext* context);
#endif

  /** Cancel the active ClientContext if any. */
  void cancel();

 private:
#if AUTOVIZ_ENABLE_GRPC
  std::shared_ptr<grpc::Channel> channel_;
  mutable std::mutex context_mu_;
  grpc::ClientContext* active_context_ = nullptr;
#endif
};

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
