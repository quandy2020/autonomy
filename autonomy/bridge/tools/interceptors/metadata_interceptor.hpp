/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file metadata_interceptor.hpp
 * @brief Required-metadata enforcement interceptor.
 */

#pragma once

#include <memory>

#include "autonomy/bridge/policy/metadata/metadata_validator.hpp"
#include "autonomy/bridge/tools/interceptors/interceptor_utils.hpp"
#include "autonomy/bridge/tools/interceptors/rejecting_interceptor.hpp"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Enforces MetadataValidator on incoming metadata.
 */
class MetadataInterceptor : public RejectingInterceptor
{
public:
    MetadataInterceptor(
        ::grpc::experimental::ServerRpcInfo* info,
        std::shared_ptr<policy::MetadataValidator> validator)
        : RejectingInterceptor(info), validator_(std::move(validator)) {}

protected:
    void MaybeReject(
        ::grpc::experimental::InterceptorBatchMethods* methods) override {
        if (!validator_) {
            return;
        }
        const auto flat = FlattenMetadata(methods->GetRecvInitialMetadata());
        const auto result = validator_->Validate(flat);
        if (!result.ok) {
            Reject(::grpc::StatusCode::FAILED_PRECONDITION, result.detail);
        }
    }

private:
    std::shared_ptr<policy::MetadataValidator> validator_;
};

class MetadataInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface
{
public:
    explicit MetadataInterceptorFactory(
        std::shared_ptr<policy::MetadataValidator> validator)
        : validator_(std::move(validator)) {}

    ::grpc::experimental::Interceptor* CreateServerInterceptor(
        ::grpc::experimental::ServerRpcInfo* info) override {
        return new MetadataInterceptor(info, validator_);
    }

private:
    std::shared_ptr<policy::MetadataValidator> validator_;
};

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
