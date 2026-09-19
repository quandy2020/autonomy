/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rejecting_interceptor.hpp
 * @brief Base interceptor that can reject with a gRPC Status.
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "grpc++/grpc++.h"
#include "grpcpp/support/interceptor.h"
#include "grpcpp/support/server_interceptor.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Server interceptor that may reject on POST_RECV_INITIAL_METADATA.
 *
 * Rejection sets PRE_SEND_STATUS via ModifySendStatus and TryCancel so the
 * RPC does not complete successfully. Handler may still be scheduled under
 * async_grpc; status override is the enforceable contract for phase 1.
 */
class RejectingInterceptor : public ::grpc::experimental::Interceptor
{
public:
    explicit RejectingInterceptor(::grpc::experimental::ServerRpcInfo* info)
        : info_(info) {}

    void Intercept(
        ::grpc::experimental::InterceptorBatchMethods* methods) override {
        using HP = ::grpc::experimental::InterceptionHookPoints;
        if (methods->QueryInterceptionHookPoint(HP::POST_RECV_INITIAL_METADATA)) {
            MaybeReject(methods);
        }
        if (rejected_ &&
            methods->QueryInterceptionHookPoint(HP::PRE_SEND_STATUS)) {
            methods->ModifySendStatus(reject_status_);
        }
        methods->Proceed();
    }

protected:
    /**
     * @brief Subclasses inspect metadata and call Reject() if needed.
     */
    virtual void MaybeReject(
        ::grpc::experimental::InterceptorBatchMethods* methods) = 0;

    void Reject(::grpc::StatusCode code, const std::string& detail) {
        rejected_ = true;
        reject_status_ = ::grpc::Status(code, detail);
        if (info_ != nullptr && info_->server_context() != nullptr) {
            info_->server_context()->TryCancel();
        }
    }

    ::grpc::experimental::ServerRpcInfo* info_{nullptr};

private:
    bool rejected_{false};
    ::grpc::Status reject_status_;
};

/**
 * @brief Factory helper that always creates @tparam InterceptorT.
 */
template <typename InterceptorT>
class SimpleInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface
{
public:
    template <typename... Args>
    explicit SimpleInterceptorFactory(Args&&... args)
        : creator_([args...](::grpc::experimental::ServerRpcInfo* info) {
              return new InterceptorT(info, args...);
          }) {}

    ::grpc::experimental::Interceptor* CreateServerInterceptor(
        ::grpc::experimental::ServerRpcInfo* info) override {
        return creator_(info);
    }

private:
    std::function<::grpc::experimental::Interceptor*(
        ::grpc::experimental::ServerRpcInfo*)>
        creator_;
};

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
