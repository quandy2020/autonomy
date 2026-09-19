/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file tls_options.hpp
 * @brief TLS path bundle extracted from GrpcOptions.
 */

#pragma once

#include <string>

#include "autonomy/bridge/proto/grpc_options.pb.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace policy {

/**
 * @brief Filesystem paths for server TLS / mTLS.
 */
struct TlsOptions {
    AUTONOMY_SMART_PTR_DEFINITIONS(TlsOptions)

    bool enabled{false};
    std::string cert_path;
    std::string key_path;
    std::string ca_path;
    bool require_client_cert{false};

    /** @brief Build from GrpcOptions. */
    static TlsOptions FromGrpcOptions(const proto::GrpcOptions& options) {
        TlsOptions tls;
        tls.enabled = options.enable_ssl_encryption();
        tls.cert_path = options.tls_cert_path();
        tls.key_path = options.tls_key_path();
        tls.ca_path = options.tls_ca_path();
        tls.require_client_cert = options.tls_require_client_cert();
        return tls;
    }
};

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
