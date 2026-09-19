/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/auth/token_authenticator.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace policy {
namespace {

proto::GrpcOptions MakeOptions(proto::AuthMode mode,
                               const std::string& token = "") {
    proto::GrpcOptions options;
    options.set_auth_mode(mode);
    options.set_bearer_token(token);
    return options;
}

TEST(TokenAuthenticatorTest, NoneModeAlwaysOk) {
    TokenAuthenticator auth(MakeOptions(proto::AUTH_MODE_NONE, "secret"));
    EXPECT_TRUE(auth.Authenticate("").ok);
    EXPECT_TRUE(auth.Authenticate("Bearer wrong").ok);
}

TEST(TokenAuthenticatorTest, TlsModeAlwaysOk) {
    TokenAuthenticator auth(MakeOptions(proto::AUTH_MODE_TLS));
    EXPECT_TRUE(auth.Authenticate("").ok);
}

TEST(TokenAuthenticatorTest, BearerEmptyRejected) {
    TokenAuthenticator auth(
        MakeOptions(proto::AUTH_MODE_BEARER_TOKEN, "secret"));
    const auto result = auth.Authenticate("");
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.detail, "missing or invalid bearer token");
}

TEST(TokenAuthenticatorTest, BearerWrongRejected) {
    TokenAuthenticator auth(
        MakeOptions(proto::AUTH_MODE_BEARER_TOKEN, "secret"));
    EXPECT_FALSE(auth.Authenticate("Bearer other").ok);
}

TEST(TokenAuthenticatorTest, BearerCorrectAccepted) {
    TokenAuthenticator auth(
        MakeOptions(proto::AUTH_MODE_BEARER_TOKEN, "secret"));
    EXPECT_TRUE(auth.Authenticate("Bearer secret").ok);
}

}  // namespace
}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
