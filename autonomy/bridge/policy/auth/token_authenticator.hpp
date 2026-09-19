/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file token_authenticator.hpp
 * @brief Bearer-token authentication helper for Bridge gRPC.
 */

#pragma once

#include <string>

#include "autonomy/bridge/proto/grpc_options.pb.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace policy {

/**
 * @brief Result of an authentication attempt.
 */
struct AuthResult {
    AUTONOMY_SMART_PTR_DEFINITIONS(AuthResult)

    bool ok{true};
    std::string detail;
};

/**
 * @brief Validates authorization metadata according to AuthMode.
 */
class TokenAuthenticator
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TokenAuthenticator)

    /**
     * @brief Construct from GrpcOptions (reads auth_mode + token fields).
     */
    explicit TokenAuthenticator(const proto::GrpcOptions& options);

    /**
     * @brief Authenticate using the raw `authorization` header value.
     *
     * @param[in] authorization_header Full header (e.g. "Bearer secret").
     */
    AuthResult Authenticate(const std::string& authorization_header) const;

    /** @brief Configured auth mode. */
    proto::AuthMode mode() const { return mode_; }

private:
    proto::AuthMode mode_{proto::AUTH_MODE_NONE};
    std::string expected_token_;
};

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
