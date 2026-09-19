/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file logging_interceptor.hpp
 * @brief Best-effort RPC logging interceptor.
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/common/log.hpp"
#include "autonomy/bridge/policy/metadata/metadata_keys.hpp"
#include "autonomy/bridge/tools/interceptors/interceptor_utils.hpp"
#include "grpcpp/support/interceptor.h"
#include "grpcpp/support/server_interceptor.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Logs method name and optional x-request-id on recv metadata.
 */
class LoggingInterceptor : public ::grpc::experimental::Interceptor
{
public:
    explicit LoggingInterceptor(::grpc::experimental::ServerRpcInfo* info)
        : info_(info) {}

    void Intercept(
        ::grpc::experimental::InterceptorBatchMethods* methods) override {
        using HP = ::grpc::experimental::InterceptionHookPoints;
        if (methods->QueryInterceptionHookPoint(
                HP::POST_RECV_INITIAL_METADATA)) {
            const auto flat =
                FlattenMetadata(methods->GetRecvInitialMetadata());
            const std::string request_id =
                LookupMetadata(flat, policy::kMetadataKeyRequestId);
            const char* method =
                (info_ != nullptr && info_->method() != nullptr) ? info_->method()
                                                                 : "?";
            if (request_id.empty()) {
                AINFO << "gRPC " << method;
            } else {
                AINFO << "gRPC " << method << " x-request-id=" << request_id;
            }
        }
        methods->Proceed();
    }

private:
    ::grpc::experimental::ServerRpcInfo* info_{nullptr};
};

class LoggingInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface
{
public:
    ::grpc::experimental::Interceptor* CreateServerInterceptor(
        ::grpc::experimental::ServerRpcInfo* info) override {
        return new LoggingInterceptor(info);
    }
};

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
