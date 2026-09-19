/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file credentials_factory.hpp
 * @brief Create ServerCredentials from GrpcOptions / TlsOptions.
 */

#pragma once

#include <memory>

#include "autonomy/bridge/proto/grpc_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "grpc++/grpc++.h"

namespace autonomy {
namespace bridge {
namespace tools {

/**
 * @brief Builds insecure or TLS server credentials.
 */
class CredentialsFactory
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(CredentialsFactory)

    /**
     * @brief Create credentials; falls back to insecure on TLS misconfig.
     */
    static std::shared_ptr<::grpc::ServerCredentials> Create(
        const proto::GrpcOptions& options);
};

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
