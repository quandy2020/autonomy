/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file auth_interceptor.hpp
 * @brief Bearer-token auth interceptor.
 */

#pragma once

#include <memory>

#include "autonomy/bridge/policy/auth/token_authenticator.hpp"
#include "autonomy/bridge/policy/metadata/metadata_keys.hpp"
#include "autonomy/bridge/tools/interceptors/interceptor_utils.hpp"
#include "autonomy/bridge/tools/interceptors/rejecting_interceptor.hpp"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Enforces TokenAuthenticator on incoming metadata.
 */
class AuthInterceptor : public RejectingInterceptor
{
public:
    AuthInterceptor(::grpc::experimental::ServerRpcInfo* info,
                    std::shared_ptr<policy::TokenAuthenticator> authenticator)
        : RejectingInterceptor(info),
          authenticator_(std::move(authenticator)) {}

protected:
    void MaybeReject(
        ::grpc::experimental::InterceptorBatchMethods* methods) override {
        if (!authenticator_ ||
            authenticator_->mode() == proto::AUTH_MODE_NONE ||
            authenticator_->mode() == proto::AUTH_MODE_TLS) {
            return;
        }
        const auto flat = FlattenMetadata(methods->GetRecvInitialMetadata());
        const auto auth =
            LookupMetadata(flat, policy::kMetadataKeyAuthorization);
        const auto result = authenticator_->Authenticate(auth);
        if (!result.ok) {
            Reject(::grpc::StatusCode::UNAUTHENTICATED, result.detail);
        }
    }

private:
    std::shared_ptr<policy::TokenAuthenticator> authenticator_;
};

class AuthInterceptorFactory
    : public ::grpc::experimental::ServerInterceptorFactoryInterface
{
public:
    explicit AuthInterceptorFactory(
        std::shared_ptr<policy::TokenAuthenticator> authenticator)
        : authenticator_(std::move(authenticator)) {}

    ::grpc::experimental::Interceptor* CreateServerInterceptor(
        ::grpc::experimental::ServerRpcInfo* info) override {
        return new AuthInterceptor(info, authenticator_);
    }

private:
    std::shared_ptr<policy::TokenAuthenticator> authenticator_;
};

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
