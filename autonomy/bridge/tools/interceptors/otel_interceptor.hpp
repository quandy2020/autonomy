/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file otel_interceptor.hpp
 * @brief OpenTelemetry span interceptor (no-op provider by default).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/bridge/tools/otel/tracer_provider.hpp"
#include "grpcpp/support/interceptor.h"
#include "grpcpp/support/server_interceptor.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Starts a span around the RPC using TracerProvider.
 */
class OtelInterceptor : public ::grpc::experimental::Interceptor
{
public:
    OtelInterceptor(::grpc::experimental::ServerRpcInfo* info,
                    std::shared_ptr<otel::TracerProvider> provider)
        : info_(info), provider_(std::move(provider)) {
        if (provider_) {
            const char* method =
                (info_ != nullptr && info_->method() != nullptr) ? info_->method()
                                                                 : "rpc";
            span_ = provider_->StartSpan(method);
        }
    }

    void Intercept(
        ::grpc::experimental::InterceptorBatchMethods* methods) override {
        using HP = ::grpc::experimental::InterceptionHookPoints;
        if (span_ &&
            methods->QueryInterceptionHookPoint(HP::PRE_SEND_STATUS)) {
            const auto status = methods->GetSendStatus();
            if (status.ok()) {
                span_->SetStatusOk();
            } else {
                span_->SetStatusError(status.error_message());
            }
        }
        methods->Proceed();
    }

private:
    ::grpc::experimental::ServerRpcInfo* info_{nullptr};
    std::shared_ptr<otel::TracerProvider> provider_;
    std::unique_ptr<otel::Span> span_;
};

class OtelInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface
{
public:
    explicit OtelInterceptorFactory(
        std::shared_ptr<otel::TracerProvider> provider)
        : provider_(std::move(provider)) {}

    ::grpc::experimental::Interceptor* CreateServerInterceptor(
        ::grpc::experimental::ServerRpcInfo* info) override {
        return new OtelInterceptor(info, provider_);
    }

private:
    std::shared_ptr<otel::TracerProvider> provider_;
};

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
