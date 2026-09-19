/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/auth/token_authenticator.hpp"

#include <fstream>
#include <sstream>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace bridge {
namespace policy {
namespace {

std::string ReadFileTrimmed(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        AERROR << "TokenAuthenticator: cannot read bearer_token_file=" << path;
        return {};
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    std::string value = ss.str();
    while (!value.empty() &&
           (value.back() == '\n' || value.back() == '\r' || value.back() == ' ')) {
        value.pop_back();
    }
    return value;
}

}  // namespace

TokenAuthenticator::TokenAuthenticator(const proto::GrpcOptions& options)
    : mode_(options.auth_mode()) {
    if (!options.bearer_token_file().empty()) {
        expected_token_ = ReadFileTrimmed(options.bearer_token_file());
    } else {
        expected_token_ = options.bearer_token();
    }
}

AuthResult TokenAuthenticator::Authenticate(
    const std::string& authorization_header) const {
    AuthResult result;
    if (mode_ == proto::AUTH_MODE_NONE || mode_ == proto::AUTH_MODE_TLS) {
        return result;
    }
    static constexpr char kPrefix[] = "Bearer ";
    if (authorization_header.size() <= sizeof(kPrefix) - 1 ||
        authorization_header.compare(0, sizeof(kPrefix) - 1, kPrefix) != 0) {
        result.ok = false;
        result.detail = "missing or invalid bearer token";
        return result;
    }
    const std::string token =
        authorization_header.substr(sizeof(kPrefix) - 1);
    if (expected_token_.empty() || token != expected_token_) {
        result.ok = false;
        result.detail = "missing or invalid bearer token";
    }
    return result;
}

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
