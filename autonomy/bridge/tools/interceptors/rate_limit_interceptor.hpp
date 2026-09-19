/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file rate_limit_interceptor.hpp
 * @brief Token-bucket rate limit interceptor.
 */

#pragma once

#include <memory>

#include "autonomy/bridge/policy/rate_limit/token_bucket.hpp"
#include "autonomy/bridge/tools/interceptors/rejecting_interceptor.hpp"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Enforces a shared TokenBucket across RPCs.
 */
class RateLimitInterceptor : public RejectingInterceptor
{
public:
    RateLimitInterceptor(::grpc::experimental::ServerRpcInfo* info,
                         std::shared_ptr<policy::TokenBucket> bucket)
        : RejectingInterceptor(info), bucket_(std::move(bucket)) {}

protected:
    void MaybeReject(
        ::grpc::experimental::InterceptorBatchMethods* /*methods*/) override {
        if (!bucket_ || !bucket_->enabled()) {
            return;
        }
        if (!bucket_->TryAcquire()) {
            Reject(::grpc::StatusCode::RESOURCE_EXHAUSTED,
                   "rate limit exceeded");
        }
    }

private:
    std::shared_ptr<policy::TokenBucket> bucket_;
};

class RateLimitInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface
{
public:
    explicit RateLimitInterceptorFactory(
        std::shared_ptr<policy::TokenBucket> bucket)
        : bucket_(std::move(bucket)) {}

    ::grpc::experimental::Interceptor* CreateServerInterceptor(
        ::grpc::experimental::ServerRpcInfo* info) override {
        return new RateLimitInterceptor(info, bucket_);
    }

private:
    std::shared_ptr<policy::TokenBucket> bucket_;
};

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
